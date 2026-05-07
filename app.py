import streamlit as st
import folium
from streamlit_folium import st_folium
import json
import math
import time
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
from shapely.affinity import scale
from pyproj import Transformer

# ===================== 坐标系转换（GCJ-02 ↔ WGS84）核心 =====================
transformer_wgs84_to_gcj02 = Transformer.from_crs("EPSG:4326", "EPSG:4490", always_xy=True)
transformer_gcj02_to_wgs84 = Transformer.from_crs("EPSG:4490", "EPSG:4326", always_xy=True)

def wgs84_to_gcj02(lon, lat):
    return transformer_wgs84_to_gcj02.transform(lon, lat)

def gcj02_to_wgs84(lon, lat):
    return transformer_gcj02_to_wgs84.transform(lon, lat)

# ===================== 初始化 =====================
defaults = {
    "point_a": None,
    "point_b": None,
    "obstacles_all": [],
    "obstacles_height": [],
    "obstacle_names": [],
    "drone_height": 8,
    "drone_safety_radius": 15,
    "avoid_mode": "向左绕飞",
    "flight_path": [],
    "drone_pos": None,
    "flight_idx": 0,
    "heading": 0,
    "drawing_mode": False,
    "temp_points": [],
    "heartbeat_log": [],
    "tab": "航线规划"
}

for k, v in defaults.items():
    if k not in st.session_state:
        st.session_state[k] = v

# ===================== 平滑轨迹 =====================
def interpolate_path(path, steps=15):
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

# ===================== 航向角 =====================
def get_heading(lat1, lng1, lat2, lng2):
    dx = lng2 - lng1
    dy = lat2 - lat1
    angle = math.degrees(math.atan2(dx, dy))
    return angle

# ===================== 绕飞算法（左/右/最短弧线） =====================
def calculate_route():
    A = st.session_state.point_a
    B = st.session_state.point_b
    if not A or not B:
        return []
    drone_h = st.session_state.drone_height
    safety_r = st.session_state.drone_safety_radius
    mode = st.session_state.avoid_mode
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

            if mode == "向左绕飞":
                wx, wy = -dy, dx
            elif mode == "向右绕飞":
                wx, wy = dy, -dx
            else:
                wx, wy = dx, dy

            route.append((px + wx * OFFSET, py + wy * OFFSET))
        except:
            continue
    route.append(B)
    return route

# ===================== 界面：多标签页 =====================
st.set_page_config(layout="wide")
tab1, tab2 = st.tabs(["✅ 航线规划", "📡 飞行监控（心跳包）"])

with tab1:
    st.title("📌 无人机航线规划系统（GCJ-02坐标系）")
    col_left, col_right = st.columns([1, 3])

    with col_left:
        st.subheader("📍 坐标输入（校园内）")
        a_lat = st.number_input("A点纬度", value=32.2330, format="%.6f")
        a_lng = st.number_input("A点经度", value=118.7490, format="%.6f")
        b_lat = st.number_input("B点纬度", value=32.2340, format="%.6f")
        b_lng = st.number_input("B点经度", value=118.7500, format="%.6f")

        if st.button("✅ 应用A/B点"):
            st.session_state.point_a = (a_lat, a_lng)
            st.session_state.point_b = (b_lat, b_lng)

        st.subheader("🛠️ 飞行参数")
        st.session_state.drone_height = st.slider("无人机高度(m)", 0, 150, 8)
        st.session_state.drone_safety_radius = st.slider("安全半径(m)", 1, 50, 15)
        st.session_state.avoid_mode = st.radio("绕飞模式", ["向左绕飞", "向右绕飞", "最短弧线"])

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
            if st.button("⏹️ 停止"):
                st.session_state.drone_pos = None

        st.subheader("🟥 障碍物圈选（记忆）")
        h = st.slider("障碍物高度(m)", 5, 150, 30)
        if st.button("🟢 开始圈选"):
            st.session_state.drawing_mode = True
            st.session_state.temp_points = []
        if st.button("✅ 完成圈选并保存"):
            if len(st.session_state.temp_points) >= 3:
                st.session_state.temp_points.append(st.session_state.temp_points[0])
                st.session_state.obstacles_all.append(st.session_state.temp_points)
                st.session_state.obstacles_height.append(h)
                st.session_state.obstacle_names.append(f"障碍{len(st.session_state.obstacles_all)}")
                st.success(f"已保存障碍，高度={h}m")
        if st.button("🗑️ 清空所有障碍物"):
            st.session_state.obstacles_all = []
            st.session_state.obstacles_height = []
            st.session_state.obstacle_names = []

    with col_right:
        st.caption("🗺️ 卫星地图（OpenStreet + GCJ-02），放大后为2D便于圈选")
        route = calculate_route()
        center = st.session_state.point_a or (32.2330, 118.7490)
        m = folium.Map(
            location=center,
            zoom_start=19,
            tiles="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
            attr="OpenStreetMap"
        )

        if st.session_state.point_a:
            folium.Marker(st.session_state.point_a, icon=folium.Icon(color="green"), tooltip="A点").add_to(m)
        if st.session_state.point_b:
            folium.Marker(st.session_state.point_b, icon=folium.Icon(color="red"), tooltip="B点").add_to(m)

        for obs in st.session_state.obstacles_all:
            if len(obs) > 2:
                folium.Polygon(obs, color="red", fill=True, fill_opacity=0.5).add_to(m)

        if len(route) >= 2:
            folium.PolyLine(route, color="blue", weight=4, dash_array="10,10").add_to(m)

        if st.session_state.drone_pos:
            lat, lng = st.session_state.drone_pos
            icon_html = f'<div style="transform:rotate({st.session_state.heading}deg);font-size:28px;">✈️</div>'
            folium.Marker((lat, lng), icon=folium.DivIcon(html=icon_html)).add_to(m)

        map_out = st_folium(m, height=750, key="map", returned_objects=["last_clicked"])

        # 圈选点采集
        if map_out and map_out.get("last_clicked"):
            lat = map_out["last_clicked"]["lat"]
            lng = map_out["last_clicked"]["lng"]
            if st.session_state.drawing_mode:
                st.session_state.temp_points.append([lat, lng])

with tab2:
    st.title("📡 飞行监控 & 心跳包")
    st.subheader("实时无人机状态")
    if st.session_state.drone_pos:
        lat, lng = st.session_state.drone_pos
        st.metric("当前位置", f"Lat:{lat:.5f} Lng:{lng:.5f}")
        st.metric("飞行高度", f"{st.session_state.drone_height} m")
        st.metric("安全半径", f"{st.session_state.drone_safety_radius} m")
        st.metric("绕飞模式", st.session_state.avoid_mode)
    else:
        st.info("未起飞，无实时数据")

    st.subheader("🧠 心跳包日志")
    now = time.strftime("%H:%M:%S")
    if st.session_state.drone_pos:
        st.session_state.heartbeat_log.append(f"[{now}] 在线 | 位置有效")
    else:
        st.session_state.heartbeat_log.append(f"[{now}] 待机")

    for line in st.session_state.heartbeat_log[-15:]:
        st.text(line)

# ===================== 飞行循环 =====================
if st.session_state.drone_pos and st.session_state.flight_path:
    total = len(st.session_state.flight_path)
    if st.session_state.flight_idx < total - 1:
        st.session_state.flight_idx += 1
        st.session_state.drone_pos = st.session_state.flight_path[st.session_state.flight_idx]
        if st.session_state.flight_idx + 1 < total:
            p1 = st.session_state.flight_path[st.session_state.flight_idx]
            p2 = st.session_state.flight_path[st.session_state.flight_idx + 1]
            st.session_state.heading = get_heading(p1[0], p1[1], p2[0], p2[1])
        time.sleep(0.08)
        st.rerun()
    else:
        st.success("✅ 已到达目的地")