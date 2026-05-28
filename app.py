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
    dlng = (dlng * 180.0) / (a / sqrtmagic * magic * radlat)
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
    dlng = (dlng * 180.0) / (a / sqrtmagic * magic * radlat)
    return [lng * 2 - (lng + dlng), lat * 2 - (lat + dlat)]

# ================== 初始化会话状态 ==================
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
    st.session_state.drawing_mode = None          # 当前绘制的障碍物类型，None表示未绘制
if "current_points" not in st.session_state:
    st.session_state.current_points = []          # 当前绘制的顶点列表
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

# 飞行动画状态
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

# ================== 平滑航线插值 ==================
def interpolate_path(path, steps=120):
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

# ================== 核心：绕飞算法 ==================
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
    st.session_state.heartbeat_log.append({
        "time": now.strftime("%Y-%m-%d %H:%M:%S"),
        "seq": st.session_state.drone_heartbeat["heartbeat_seq"]
    })

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
    st.session_state.drone_height = st.slider("米",0,200,8)

    st.markdown("---")
    st.subheader("🛡️ 安全半径")
    st.session_state.drone_safety_radius = st.slider("安全距离（米）",1,50,15)

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

    # 飞行动画控制
    st.markdown("---")
    st.subheader("✈️ 无人机飞行")
    col_play, col_stop = st.columns(2)
    with col_play:
        if st.button("▶️ 开始飞行", disabled=st.session_state.is_flying):
            # 计算航线
            route, _ = calculate_route_with_direction()
            if len(route) >= 2:
                # 减少插值步数（30步），减少刷新次数，降低闪烁
                st.session_state.flight_path = interpolate_path(route, steps=30)
                st.session_state.is_flying = True
                st.session_state.flight_step = 0
                st.session_state.drone_pos = route[0]   # 起点
                st.rerun()
            else:
                st.error("请先设置 A 点和 B 点")
    with col_stop:
        if st.button("⏹️ 停止飞行"):
            st.session_state.is_flying = False
            st.session_state.drone_pos = None
            st.session_state.flight_step = 0
            st.rerun()

    # ================== 障碍物绘制区域（修复后） ==================
    st.markdown("---")
    st.subheader("🌍 障碍物绘制")
    dtype = st.selectbox("障碍物类型", ["自定义障碍物","普通房屋","高层楼房","大树/电线杆","操场/空地","桥梁/高架","塔楼/信号塔"])

    # ========== 新增：障碍物高度设置（独立于类型） ==========
    default_h = REAL_WORLD_HEIGHTS.get(dtype, 50)
    obstacle_custom_height = st.number_input(
        "障碍物高度（米）",
        min_value=0,
        max_value=200,
        value=default_h,
        step=1,
        help="设置当前要绘制的障碍物的实际高度。超过无人机飞行高度时会触发绕飞。"
    )
    # ====================================================

    # 显示当前绘制状态
    if st.session_state.drawing_mode:
        st.info(f"✏️ 正在绘制：{st.session_state.drawing_mode} （已选 {len(st.session_state.current_points)} 个点）")
    else:
        st.info("⚙️ 未绘制，点击「开始圈选」")
    
    col_draw, col_cancel, col_save = st.columns(3)
    with col_draw:
        if st.button("🟢 开始圈选"):
            st.session_state.drawing_mode = dtype
            st.session_state.current_points = []
            st.rerun()
    with col_cancel:
        if st.button("❌ 取消圈选"):
            st.session_state.drawing_mode = None
            st.session_state.current_points = []
            st.rerun()
    with col_save:
        if st.button("✅ 完成圈选"):
            if st.session_state.drawing_mode and len(st.session_state.current_points) >= 3:
                closed_points = st.session_state.current_points + [st.session_state.current_points[0]]
                st.session_state.obstacles_all.append(closed_points)
                st.session_state.obstacles_type.append(st.session_state.drawing_mode)
                st.session_state.obstacles_height.append(obstacle_custom_height)
                save_all()
            
                # 同步更新转换后的障碍物列表（如果用户之前转换过坐标系）
                if st.session_state.coord_system != "WGS84（原始GPS）":
                    def conv_obstacle(p):
                        if not p: return None
                        lat, lng = p
                        if st.session_state.coord_system == "GCJ02（火星坐标系）":
                            nlng, nlat = wgs84_to_gcj02(lng, lat)
                        else:
                            nlng, nlat = gcj02_to_wgs84(lng, lat)
                        return [round(nlat,6), round(nlng,6)]
                    transformed_obs = [conv_obstacle(pt) for pt in closed_points]
                    if st.session_state.transformed_points["obstacles"] is None:
                        st.session_state.transformed_points["obstacles"] = []
                    st.session_state.transformed_points["obstacles"].append(transformed_obs)
                else:
                    if st.session_state.transformed_points["obstacles"] is None:
                        st.session_state.transformed_points["obstacles"] = []
                    st.session_state.transformed_points["obstacles"].append(closed_points)
            
                st.session_state.drawing_mode = None
                st.session_state.current_points = []
                st.success("障碍物已保存并退出绘制模式")
            else:
                st.error("请至少绘制3个点形成多边形")
            st.rerun()
    # 新增：清除最后一个障碍物
    if st.button("🗑️ 清除最后一个障碍物"):
        if st.session_state.obstacles_all:
            st.session_state.obstacles_all.pop()
            st.session_state.obstacles_type.pop()
            st.session_state.obstacles_height.pop()
            # 同步删除转换后的列表中的最后一个
            if st.session_state.transformed_points["obstacles"]:
                st.session_state.transformed_points["obstacles"].pop()
            save_all()
            st.success("已清除最后一个障碍物")
        else:
            st.warning("没有障碍物可清除")
        st.rerun()

    st.markdown("---")
    st.subheader("📍 A / B 点")
    colA, colB = st.columns(2)
    with colA:
        if st.button("清除A点"):
            st.session_state.point_a = None
            st.session_state.transformed_points["point_a"] = None
            st.success("A点已清除")
    with colB:
        if st.button("清除B点"):
            st.session_state.point_b = None
            st.session_state.transformed_points["point_b"] = None
            st.success("B点已清除")

# ================== 航线规划页面（含地图交互） ==================
if page == "航线规划":
    st.title("🗺️ 无人机避障航线")
    route, status = calculate_route_with_direction()
    st.subheader(status)

    # 飞行动画帧推进（优化闪烁）
    if st.session_state.is_flying:
        path = st.session_state.flight_path
        step = st.session_state.flight_step
        if path and step < len(path):
            st.session_state.drone_pos = path[step]
            st.session_state.flight_step = step + 1
            # 增加延时到 0.1 秒，减少刷新频率
            time.sleep(0.1)
            st.rerun()
        else:
            # 飞行结束
            st.session_state.is_flying = False
            st.session_state.drone_pos = None
            st.session_state.flight_step = 0
            st.toast("✅ 飞行完成", icon="✈️")   # 轻提示，不干扰地图

    # 构建地图
    center = [32.232232, 118.748055]
    m = folium.Map(location=center, zoom_start=18, tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", attr="Esri")

    # 显示 A/B 点
    A = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    B = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    if A:
        folium.CircleMarker(A, radius=12, color='green', fill=True, popup="起点A").add_to(m)
    if B:
        folium.CircleMarker(B, radius=12, color='red', fill=True, popup="终点B").add_to(m)

    # 显示已保存的障碍物（多边形）
    obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    for o in obstacles:
        if len(o) > 2:
            folium.Polygon(o, color='orange', fill=True, fill_opacity=0.4).add_to(m)

    # 显示正在绘制的临时多边形（半透明预览）
    if st.session_state.drawing_mode and len(st.session_state.current_points) >= 2:
        # 为了视觉效果，可以显示一个临时多边形（未闭合）
        # 为了预览，我们显示线 + 点，不闭合
        folium.PolyLine(st.session_state.current_points, color='cyan', weight=3, opacity=0.8).add_to(m)
        for pt in st.session_state.current_points:
            folium.CircleMarker(pt, radius=4, color='blue', fill=True).add_to(m)

    # 显示航线
    if len(route) >= 2:
        folium.PolyLine(route, color='blue', weight=6, opacity=0.9).add_to(m)

    # 显示无人机当前位置
    if st.session_state.drone_pos:
        folium.CircleMarker(
            location=st.session_state.drone_pos,
            radius=8,
            color='blue',
            fill=True,
            fill_color='blue',
            fill_opacity=0.8,
            popup="无人机"
        ).add_to(m)

    # 地图点击回调
    out = st_folium(m, height=750, key="real_map", returned_objects=["last_clicked"])

    if out and out.get("last_clicked") and not st.session_state.is_flying:
        lat = out["last_clicked"]["lat"]
        lng = out["last_clicked"]["lng"]
        # 优先级：如果正在绘制障碍物 -> 添加顶点；否则 -> 设置A/B点
        if st.session_state.drawing_mode:
            st.session_state.current_points.append([lat, lng])
            st.rerun()   # 立刻刷新地图显示新点
        else:
            if not st.session_state.point_a:
                st.session_state.point_a = (lat, lng)
                st.success("✅ A点已设置")
                st.rerun()
            elif not st.session_state.point_b:
                st.session_state.point_b = (lat, lng)
                st.success("✅ B点已设置")
                st.rerun()

# ================== 飞行监控页面 ==================
else:
    st.title("📡 飞行监控")
    c1,c2,c3 = st.columns(3)
    with c1:
        if st.button("▶️ 开始监控"):
            st.session_state.heartbeat_running = True
    with c2:
        if st.button("⏹️ 停止监控"):
            st.session_state.heartbeat_running = False
    with c3:
        if st.button("🔄 重置数据"):
            st.session_state.heartbeat_log = []
            st.session_state.drone_heartbeat["heartbeat_seq"] = 0

    update_heartbeat()
    st.plotly_chart(draw_heartbeat_chart(), use_container_width=True)
    
    st.subheader("📜 最新日志")
    for log in reversed(st.session_state.heartbeat_log[-10:]):
        st.text(f"[{log['time']}] 心跳 {log['seq']}")