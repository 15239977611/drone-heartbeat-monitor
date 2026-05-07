import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point
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

# 飞行状态（和截图完全一致）
if "flight_path" not in st.session_state:
    st.session_state.flight_path = []
if "drone_pos" not in st.session_state:
    st.session_state.drone_pos = None
if "flight_idx" not in st.session_state:
    st.session_state.flight_idx = 0

# 障碍物类型高度（和截图里的房子对应）
REAL_WORLD_HEIGHTS = {
    "普通房屋": 20,
    "高层楼房": 80,
    "自定义障碍物": 30
}

# ===================== 平滑轨迹（和截图的蓝色虚线一样） =====================
def interpolate_path(path, steps=10):
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

# ===================== 绕飞算法（和截图的绕飞效果完全一致） =====================
def calculate_route():
    A = st.session_state.point_a
    B = st.session_state.point_b
    if not A or not B:
        return []

    drone_h = st.session_state.drone_height
    safety_r = st.session_state.drone_safety_radius
    direction = st.session_state.avoid_direction
    OFFSET = safety_r / 12000.0
    route = [A]

    for i, coords in enumerate(st.session_state.obstacles_all):
        if len(coords) < 3:
            continue
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 30
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

# ===================== 界面（和你的作业截图风格一致） =====================
st.set_page_config(layout="wide")
st.title("无人机智能化应用 - 航线规划与飞行模拟")

with st.sidebar:
    st.subheader("🛸 飞行参数")
    st.session_state.drone_height = st.slider("飞行高度（米）", 0, 100, 8)
    st.session_state.drone_safety_radius = st.slider("安全半径（米）", 1, 30, 15)

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

    st.subheader("🏠 障碍物圈选（和截图里的红房子一样）")
    obs_type = st.selectbox("障碍物类型", list(REAL_WORLD_HEIGHTS.keys()))
    if st.button("🟢 开始圈选障碍物"):
        st.session_state["drawing"] = True
        st.session_state["temp_points"] = []
    if st.button("✅ 完成圈选"):
        if len(st.session_state.get("temp_points", [])) >= 3:
            pts = st.session_state.temp_points
            pts.append(pts[0])
            st.session_state.obstacles_all.append(pts)
            st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[obs_type])
            st.success("障碍物添加成功！")
    if st.button("🗑️ 清空所有障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_height = []
        st.success("已清空所有障碍物！")

    st.subheader("📍 航线起点/终点")
    if st.button("🟢 设置起点 A（绿色）"):
        st.session_state.point_a = None
    if st.button("🔴 设置终点 B（红色）"):
        st.session_state.point_b = None

# ===================== 地图绘制（和你的截图 1:1 还原） =====================
route = calculate_route()
center_loc = st.session_state.point_a if st.session_state.point_a else [32.2330, 118.7490]

# 和截图一模一样的卫星地图
m = folium.Map(
    location=center_loc,
    zoom_start=18,
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="Esri World Imagery"
)

# 绘制起点A（绿色，和截图一致）
if st.session_state.point_a:
    folium.Marker(
        location=st.session_state.point_a,
        icon=folium.Icon(color="green", icon="play"),
        popup="起点A"
    ).add_to(m)

# 绘制终点B（红色，和截图一致）
if st.session_state.point_b:
    folium.Marker(
        location=st.session_state.point_b,
        icon=folium.Icon(color="red", icon="stop"),
        popup="终点B"
    ).add_to(m)

# 绘制障碍物（红色多边形，和截图里的红房子完全一样）
for obs in st.session_state.obstacles_all:
    if len(obs) > 2:
        folium.Polygon(
            locations=obs,
            color="red",
            fill=True,
            fill_color="red",
            fill_opacity=0.5,
            weight=2
        ).add_to(m)

# 绘制航线（蓝色带点虚线，和截图的蓝色航线完全一致）
if len(route) >= 2:
    folium.PolyLine(
        locations=route,
        color="blue",
        weight=4,
        opacity=0.8,
        dash_array="10,10",
        popup="规划航线"
    ).add_to(m)
    # 航线上的点，和截图里的白点一致
    for point in route:
        folium.CircleMarker(
            location=point,
            radius=3,
            color="white",
            fill=True,
            fill_color="blue",
            fill_opacity=1
        ).add_to(m)

# 绘制无人机（和截图里的绿色飞机图标一致）
if st.session_state.drone_pos:
    lat, lng = st.session_state.drone_pos
    folium.Marker(
        location=(lat, lng),
        icon=folium.Icon(color="green", icon="plane", prefix="fa"),
        popup="无人机"
    ).add_to(m)

# 固定地图容器，彻底解决闪烁
map_container = st.empty()
with map_container:
    map_output = st_folium(m, height=700, key="demo_map", returned_objects=["last_clicked"])

# ===================== 点击设置点位 =====================
if map_output and map_output.get("last_clicked"):
    lat = map_output["last_clicked"]["lat"]
    lng = map_output["last_clicked"]["lng"]

    if st.session_state.get("drawing", False):
        st.session_state.temp_points.append([lat, lng])
    else:
        if st.session_state.point_a is None:
            st.session_state.point_a = (lat, lng)
            st.success("✅ 起点A已设置！")
        elif st.session_state.point_b is None:
            st.session_state.point_b = (lat, lng)
            st.success("✅ 终点B已设置！")

# ===================== 飞行动画（无闪烁，和截图效果一致） =====================
if st.session_state.drone_pos and st.session_state.flight_path:
    total = len(st.session_state.flight_path)
    if st.session_state.flight_idx < total - 1:
        st.session_state.flight_idx += 1
        st.session_state.drone_pos = st.session_state.flight_path[st.session_state.flight_idx]
        time.sleep(0.05)
        st.rerun()
    else:
        st.success("✅ 无人机已到达终点！")