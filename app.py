import streamlit as st
import folium
from streamlit_folium import st_folium
from shapely.geometry import Polygon, LineString
from shapely.ops import nearest_points
from shapely.affinity import scale
import math
import time

# ===================== 初始化 =====================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles_all" not in st.session_state:
    st.session_state.obstacles_all = []
if "obstacles_height" not in st.session_state:
    st.session_state.obstacles_height = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 8
if "drone_safety_radius" not in st.session_state:
    st.session_state.drone_safety_radius = 15
if "avoid_direction" not in st.session_state:
    st.session_state.avoid_direction = "左"

# 飞行状态
if "flight_path" not in st.session_state:
    st.session_state.flight_path = []
if "drone_pos" not in st.session_state:
    st.session_state.drone_pos = None
if "flight_idx" not in st.session_state:
    st.session_state.flight_idx = 0

# 地图视野记忆（消除点击偏移）
if "map_center" not in st.session_state:
    st.session_state.map_center = [32.2330, 118.7490]
if "map_zoom" not in st.session_state:
    st.session_state.map_zoom = 18

# 障碍物圈选状态
if "drawing" not in st.session_state:
    st.session_state.drawing = False
if "temp_points" not in st.session_state:
    st.session_state.temp_points = []

GROUND_HEIGHT = 0
REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50, "普通房屋": 20, "高层楼房": 80,
    "大树/电线杆": 10, "操场/空地": 0, "桥梁/高架": 15, "塔楼/信号塔": 60
}

# ===================== 平滑轨迹 =====================
def interpolate_path(path, steps=40):
    smooth = []
    for i in range(len(path)-1):
        lat1, lng1 = path[i]
        lat2, lng2 = path[i+1]
        for s in range(steps):
            f = s / steps
            lat = lat1 + (lat2 - lat1) * f
            lng = lng1 + (lng2 - lng1) * f
            smooth.append((lat, lng))
    return smooth

# ===================== 绕飞算法 =====================
def calculate_route():
    A = st.session_state.point_a
    B = st.session_state.point_b
    if not A or not B:
        return []

    drone_h = st.session_state.drone_height
    safety_r = st.session_state.drone_safety_radius
    direction = st.session_state.avoid_direction
    OFFSET = safety_r / 111000.0   # 更合理的经纬度偏移
    route = [A]

    for i, coords in enumerate(st.session_state.obstacles_all):
        if len(coords) < 3:
            continue
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        if obs_h <= drone_h:
            continue

        try:
            poly = Polygon(coords)
            safe_poly = scale(poly, 1 + safety_r/1600, 1 + safety_r/1600, origin='centroid')
            cx, cy = poly.centroid.x, poly.centroid.y
            line = LineString([route[-1], B])
            if not line.intersects(safe_poly):
                continue

            p, _ = nearest_points(line, safe_poly.boundary)
            px, py = p.x, p.y

            dx = px - cx
            dy = py - cy
            dist = math.hypot(dx, dy) or 1
            dx /= dist
            dy /= dist

            if direction == "左":
                wx, wy = -dy, dx
            else:
                wx, wy = dy, -dx

            route.append((px + wx * OFFSET, py + wy * OFFSET))
        except:
            continue

    route.append(B)
    return route

# ===================== 界面 =====================
st.set_page_config(layout="wide")
st.title("✈️ 无人机避障飞行系统（稳定版）")

with st.sidebar:
    st.subheader("🛸 无人机高度")
    st.session_state.drone_height = st.slider("飞行高度（米）", 0, 200, 8)

    st.subheader("🛡️ 安全半径")
    st.session_state.drone_safety_radius = st.slider("安全距离（米）", 1, 50, 15)

    st.subheader("↔️ 绕飞方向")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("⬅️ 向左绕飞"):
            st.session_state.avoid_direction = "左"
    with col2:
        if st.button("➡️ 向右绕飞"):
            st.session_state.avoid_direction = "右"
    st.info(f"当前绕飞：{st.session_state.avoid_direction}")

    st.subheader("✈️ 飞行控制")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始飞行"):
            route = calculate_route()
            if len(route) >= 2:
                st.session_state.flight_path = interpolate_path(route)
                st.session_state.drone_pos = st.session_state.flight_path[0]
                st.session_state.flight_idx = 0
    with c2:
        if st.button("⏹️ 停止飞行"):
            st.session_state.drone_pos = None

    st.subheader("🌍 障碍物")
    obs_type = st.selectbox("障碍物类型", list(REAL_WORLD_HEIGHTS.keys()))
    if st.button("🟢 开始圈选障碍物"):
        st.session_state.drawing = True
        st.session_state.temp_points = []
    if st.button("✅ 完成圈选"):
        if len(st.session_state.temp_points) >= 3:
            pts = st.session_state.temp_points[:]
            pts.append(pts[0])   # 闭合多边形
            st.session_state.obstacles_all.append(pts)
            st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[obs_type])
            st.session_state.drawing = False
            st.session_state.temp_points = []
            st.success("障碍物已添加")
        else:
            st.warning("至少需要3个点")
    if st.button("🗑️ 清空所有障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_height = []
        st.session_state.drawing = False
        st.session_state.temp_points = []

    st.subheader("📍 标记点位")
    if st.button("🟢 重新设置起点 A"):
        st.session_state.point_a = None
    if st.button("🔴 重新设置终点 B"):
        st.session_state.point_b = None

# ===================== 地图绘制 =====================
route = calculate_route()
m = folium.Map(
    location=st.session_state.map_center,
    zoom_start=st.session_state.map_zoom,
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="ESRI"
)

# 临时圈选预览
if st.session_state.drawing and len(st.session_state.temp_points) >= 2:
    folium.PolyLine(st.session_state.temp_points, color="red", weight=3, dash_array="5,5").add_to(m)
    for pt in st.session_state.temp_points:
        folium.CircleMarker(pt, radius=4, color="darkred", fill=True).add_to(m)

# 起点、终点
if st.session_state.point_a:
    folium.CircleMarker(st.session_state.point_a, radius=10, color="green", fill=True).add_to(m)
if st.session_state.point_b:
    folium.CircleMarker(st.session_state.point_b, radius=10, color="red", fill=True).add_to(m)

# 障碍物
for obs in st.session_state.obstacles_all:
    if len(obs) > 2:
        folium.Polygon(obs, color="orange", fill=True, fill_opacity=0.4).add_to(m)

# 航线
if len(route) >= 2:
    folium.PolyLine(route, color="blue", weight=5, opacity=0.9).add_to(m)

# 无人机
if st.session_state.drone_pos:
    lat, lng = st.session_state.drone_pos
    folium.Marker(
        location=(lat, lng),
        icon=folium.DivIcon(html='<div style="font-size:28px; color:blue;">✈️</div>')
    ).add_to(m)

# 固定容器并获取视野
map_container = st.empty()
with map_container:
    map_output = st_folium(
        m,
        height=700,
        key="map",
        returned_objects=["last_clicked", "center", "zoom"]
    )

# 更新视野（关键：防止地图跳回A点）
if map_output and map_output.get("center") and map_output.get("zoom"):
    st.session_state.map_center = [map_output["center"]["lat"], map_output["center"]["lng"]]
    st.session_state.map_zoom = map_output["zoom"]

# ===================== 点击处理 =====================
if map_output and map_output.get("last_clicked"):
    lat = map_output["last_clicked"]["lat"]
    lng = map_output["last_clicked"]["lng"]

    if st.session_state.drawing:
        st.session_state.temp_points.append([lat, lng])
    else:
        if st.session_state.point_a is None:
            st.session_state.point_a = (lat, lng)
        elif st.session_state.point_b is None:
            st.session_state.point_b = (lat, lng)

# ===================== 飞行循环（不可避免的轻微闪烁，但功能正常）=====================
if st.session_state.drone_pos and st.session_state.flight_path:
    total = len(st.session_state.flight_path)
    if st.session_state.flight_idx < total - 1:
        st.session_state.flight_idx += 1
        st.session_state.drone_pos = st.session_state.flight_path[st.session_state.flight_idx]
        time.sleep(0.04)
        st.experimental_rerun()
    else:
        st.success("✅ 无人机已到达目的地！")