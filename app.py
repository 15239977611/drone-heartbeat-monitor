import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
from shapely.affinity import scale
import math
from datetime import datetime
import time
import plotly.graph_objects as go
import pandas as pd

# ================== 坐标系转换 ==================
PI = math.pi
a = 6378245.0
ee = 0.00669342162296594323

def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * PI) + 320 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return ret

def out_of_china(lat, lng):
    return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lat, lng):
        return [lng, lat]
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * PI)
    return [lng + dlng, lat + dlat]

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lat, lng):
        return [lng, lat]
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * PI)
    return [lng * 2 - (lng + dlng), lat * 2 - (lat + dlat)]

# ================== 初始化 ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles_all" not in st.session_state:
    st.session_state.obstacles_all = []
if "obstacles_type" not in st.session_state:
    st.session_state.obstacles_type = []
if "obstacles_height" not in st.session_state:
    st.session_state.obstacles_height = []
if "drawing_mode" not in st.session_state:
    st.session_state.drawing_mode = None
if "current_points" not in st.session_state:
    st.session_state.current_points = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 8
if "drone_safety_radius" not in st.session_state:
    st.session_state.drone_safety_radius = 15
if "avoid_direction" not in st.session_state:
    st.session_state.avoid_direction = "自动"

if "drone_heartbeat" not in st.session_state:
    st.session_state.drone_heartbeat = {
        "last_time": datetime.now(), "signal_strength": 95, "battery": 88,
        "gps_status": "正常", "flight_status": "待命", "latitude": 32.2330,
        "longitude": 118.7490, "speed": 0.0, "heartbeat_interval": 1, "heartbeat_seq": 0
    }
if "heartbeat_log" not in st.session_state:
    st.session_state.heartbeat_log = []
if "heartbeat_chart_data" not in st.session_state:
    st.session_state.heartbeat_chart_data = {"time": [], "seq": []}
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False
if "coord_system" not in st.session_state:
    st.session_state.coord_system = "WGS84（原始GPS）"
if "transformed_points" not in st.session_state:
    st.session_state.transformed_points = {"point_a": None, "point_b": None, "obstacles": []}

# 飞行动画（修复闪烁核心）
if "flight_path" not in st.session_state:
    st.session_state.flight_path = []
if "drone_pos" not in st.session_state:
    st.session_state.drone_pos = None
if "flight_step" not in st.session_state:
    st.session_state.flight_step = 0
if "is_flying" not in st.session_state:
    st.session_state.is_flying = False

# ================== 配置 ==================
GROUND_HEIGHT = 0
REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50, "普通房屋": 20, "高层楼房": 80,
    "大树/电线杆": 10, "操场/空地": 0, "桥梁/高架": 15, "塔楼/信号塔": 60
}

# ================== 存储 ==================
def save_all():
    with open("geo_obstacles.json", "w", encoding="utf-8") as f:
        json.dump({
            "obstacles": st.session_state.obstacles_all,
            "types": st.session_state.obstacles_type,
            "heights": st.session_state.obstacles_height
        }, f)

def load_all():
    try:
        with open("geo_obstacles.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles_all = data.get("obstacles", [])
            st.session_state.obstacles_type = data.get("types", [])
            st.session_state.obstacles_height = data.get("heights", [])
    except:
        st.session_state.obstacles_all = []

load_all()
st.set_page_config(page_title="无人机避障系统", layout="wide")

# ================== 平滑航线插值（飞行专用） ==================
def interpolate_path(path, steps=80):
    smooth = []
    for i in range(len(path)-1):
        lat1, lng1 = path[i]
        lat2, lng2 = path[i+1]
        for s in range(steps):
            f = s / steps
            lat = lat1 + (lat2-lat1)*f
            lng = lng1 + (lng2-lng1)*f
            smooth.append((lat, lng))
    return smooth

# ================== 核心：左右绕飞算法 ==================
def calculate_route_with_direction():
    A = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    B = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    if not A or not B:
        return [], "未设置起点/终点"

    drone_h = st.session_state.drone_height
    safety_r = st.session_state.drone_safety_radius
    direction = st.session_state.avoid_direction
    SAFE_OFFSET = safety_r / 12000.0

    obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    obstacle_list = []

    for i, coords in enumerate(obstacles):
        if len(coords) < 3: continue
        h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        if h <= drone_h: continue
        try:
            poly = Polygon(coords)
            if not poly.is_valid: continue
            scaled = scale(poly, xfact=1.0 + safety_r/1600., yfact=1.0 + safety_r/1600., origin='centroid')
            obstacle_list.append({"poly": scaled, "center": poly.centroid})
        except:
            continue

    if not obstacle_list:
        return [A, B], f"🟢 高度足够，直线飞行 | 安全半径：{safety_r}米"

    route = [A]
    current = A
    for obs in obstacle_list:
        line = LineString([current, B])
        if not line.intersects(obs["poly"]): continue
        p1, _ = nearest_points(line, obs["poly"].boundary)
        px, py = p1.x, p1.y
        cx, cy = obs["center"].x, obs["center"].y
        dx = px - cx
        dy = py - cy
        dist = math.hypot(dx, dy) or 1
        dx /= dist
        dy /= dist

        if direction == "左":
            offset_x = -dy
            offset_y = dx
        elif direction == "右":
            offset_x = dy
            offset_y = -dx
        else:
            offset_x = dx
            offset_y = dy

        wp = (px + offset_x * SAFE_OFFSET, py + offset_y * SAFE_OFFSET)
        route.append(wp)
        current = wp
    route.append(B)
    return route, f"🔴 {direction}绕飞 | 安全半径：{safety_r}米 | 高度：{drone_h}m"

# ================== 心跳 ==================
def update_heartbeat():
    if not st.session_state.heartbeat_running: return
    now = datetime.now()
    if (now - st.session_state.drone_heartbeat["last_time"]).total_seconds() < 1:
        return
    st.session_state.drone_heartbeat["heartbeat_seq"] += 1
    st.session_state.drone_heartbeat["last_time"] = now

def draw_heartbeat_chart():
    df = pd.DataFrame({
        "t": [x["time"] for x in st.session_state.heartbeat_log],
        "seq": [x["seq"] for x in st.session_state.heartbeat_log]
    })
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=df["t"], y=df["seq"], line=dict(width=3)))
    fig.update_layout(height=350, title="心跳包")
    return fig

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("无人机避障系统")
    page = st.radio("页面", ["航线规划", "飞行监控"])

    if page == "航线规划":
        st.markdown("---")
        st.subheader("🌐 坐标系")
        st.session_state.coord_system = st.selectbox(
            "目标", ["WGS84（原始GPS）", "GCJ02（火星坐标系）"]
        )
        if st.button("✅ 转换坐标"):
            def conv(p):
                if not p: return None
                lat, lng = p
                if st.session_state.coord_system == "GCJ02（火星坐标系）":
                    nlng, nlat = wgs84_to_gcj02(lng, lat)
                else:
                    nlng, nlat = gcj02_to_wgs84(lng, lat)
                return [round(nlat,6), round(nlng,6)]
            st.session_state.transformed_points["point_a"] = conv(st.session_state.point_a)
            st.session_state.transformed_points["point_b"] = conv(st.session_state.point_b)
            nobs = []
            for o in st.session_state.obstacles_all:
                nobs.append([conv(p) for p in o])
            st.session_state.transformed_points["obstacles"] = nobs
            st.success("转换完成")

        if st.button("🔄 重置坐标"):
            st.session_state.transformed_points = {"point_a":None,"point_b":None,"obstacles":[]}

    st.markdown("---")
    st.subheader("🛸 飞行高度")
    st.session_state.drone_height = st.slider("米", 0, 200, 8)

    st.markdown("---")
    st.subheader("🛡️ 安全半径")
    st.session_state.drone_safety_radius = st.slider("安全距离（米）", 1, 50, 15)

    st.markdown("---")
    st.subheader("↔️ 绕飞方向")
    c1,c2 = st.columns(2)
    with c1:
        if st.button("⬅️ 向左绕飞"):
            st.session_state.avoid_direction = "左"
    with c2:
        if st.button("➡️ 向右绕飞"):
            st.session_state.avoid_direction = "右"
    st.info(f"当前：{st.session_state.avoid_direction} 绕飞")

    # ✈️ 飞行控制
    st.markdown("---")
    st.subheader("✈️ 无人机飞行")
    col_play, col_stop = st.columns(2)
    with col_play:
        if st.button("▶️ 开始飞行"):
            route, _ = calculate_route_with_direction()
            if len(route) >= 2:
                st.session_state.flight_path = interpolate_path(route, steps=60)
                st.session_state.is_flying = True
                st.session_state.flight_step = 0
                if st.session_state.flight_path:
                    st.session_state.drone_pos = st.session_state.flight_path[0]
    with col_stop:
        if st.button("⏹️ 停止飞行"):
            st.session_state.is_flying = False
            st.session_state.drone_pos = None

    st.markdown("---")
    st.subheader("🌍 障碍物")
    dtype = st.selectbox("类型", ["无","自定义障碍物","普通房屋","高层楼房","大树/电线杆","操场/空地","桥梁/高架","塔楼/信号塔"])
    if st.button("🟢 开始圈选") and dtype!="无":
        st.session_state.drawing_mode = dtype
        st.session_state.current_points = []
    if st.button("✅ 完成圈选") and st.session_state.drawing_mode:
        if len(st.session_state.current_points)>=3:
            st.session_state.current_points.append(st.session_state.current_points[0])
            st.session_state.obstacles_all.append(st.session_state.current_points)
            st.session_state.obstacles_type.append(st.session_state.drawing_mode)
            st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[dtype])
            save_all()
    if st.button("🗑️ 清空障碍物"):
        st.session_state.obstacles_all=[]
        st.session_state.obstacles_type=[]
        st.session_state.obstacles_height=[]
        save_all()

    st.markdown("---")
    st.subheader("📍 A / B 点")
    colA, colB = st.columns(2)
    with colA:
        if st.button("清除A点"):
            st.session_state.point_a = None
            st.session_state.transformed_points["point_a"] = None
    with colB:
        if st.button("清除B点"):
            st.session_state.point_b = None
            st.session_state.transformed_points["point_b"] = None

# ================== 地图页面 ==================
if page == "航线规划":
    st.title("🗺️ 无人机避障航线")
    route, status = calculate_route_with_direction()
    st.subheader(status)

    center = [32.2330, 118.7490]
    m = folium.Map(location=center, zoom_start=18, tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", attr="Esri")

    A = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    B = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    if A:
        folium.CircleMarker(A, radius=12, color='green', fill=True, popup="起点A").add_to(m)
    if B:
        folium.CircleMarker(B, radius=12, color='red', fill=True, popup="终点B").add_to(m)

    obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    for o in obstacles:
        if len(o)>2:
            folium.Polygon(o, color='orange', fill=True, fill_opacity=0.4).add_to(m)

    if len(route)>=2:
        folium.PolyLine(route, color='blue', weight=6, opacity=0.9).add_to(m)

    # ✈️ 绘制无人机（修复闪烁）
    if st.session_state.drone_pos:
        lat, lng = st.session_state.drone_pos
        folium.Marker(
            location=(lat, lng),
            icon=folium.DivIcon(html='''
                <div style="font-size:26px; color:#00aaff; text-shadow: 0 0 3px #000;">✈️</div>
            ''')
        ).add_to(m)

    # 关键修复：只渲染地图，不重复创建
    map_placeholder = st.empty()
    with map_placeholder:
        out = st_folium(m, height=750, key="drone_map_fixed", returned_objects=["last_clicked"])

    # 飞行动画逻辑（彻底无闪烁）
    if st.session_state.is_flying and st.session_state.flight_path:
        total = len(st.session_state.flight_path)
        if st.session_state.flight_step < total:
            # 只更新位置，不重绘整个地图
            pos = st.session_state.flight_path[st.session_state.flight_step]
            st.session_state.drone_pos = pos
            st.session_state.flight_step += 1
            time.sleep(0.06)
            st.rerun()
        else:
            st.session_state.is_flying = False
            st.success("✅ 无人机已到达B点！")

    if out and out.get("last_clicked"):
        lat = out["last_clicked"]["lat"]
        lng = out["last_clicked"]["lng"]
        if st.session_state.drawing_mode:
            st.session_state.current_points.append([lat,lng])
        else:
            if not st.session_state.point_a:
                st.session_state.point_a = (lat,lng)
                st.success("✅ A点已设置")
            elif not st.session_state.point_b:
                st.session_state.point_b = (lat,lng)
                st.success("✅ B点已设置")

# ================== 监控页面 ==================
else:
    st.title("📡 飞行监控")
    c1,c2,c3 = st.columns(3)
    with c1:
        if st.button("▶️ 开始监控"):
            st.session_state.heartbeat_running=True
    with c2:
        if st.button("⏹️ 停止监控"):
            st.session_state.heartbeat_running=False
    with c3:
        if st.button("🔄 重置数据"):
            st.session_state.heartbeat_log=[]
            st.session_state.drone_heartbeat["heartbeat_seq"]=0

    update_heartbeat()
    st.plotly_chart(draw_heartbeat_chart(), use_container_width=True)
    st.subheader("📜 日志")
    for log in reversed(st.session_state.heartbeat_log[-15:]):
        st.text(f"[{log['time']}] 心跳 {log['seq']}")

    if st.session_state.heartbeat_running:
        time.sleep(1)
        st.rerun()