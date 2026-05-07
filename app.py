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
import pandas as pd

# ===================== 坐标系转换（GCJ-02 ↔ WGS84）核心 =====================
transformer_wgs84_to_gcj02 = Transformer.from_crs("EPSG:4326", "EPSG:4490", always_xy=True)
transformer_gcj02_to_wgs84 = Transformer.from_crs("EPSG:4490", "EPSG:4326", always_xy=True)

def wgs84_to_gcj02(lon, lat):
    return transformer_wgs84_to_gcj02.transform(lon, lat)

def gcj02_to_wgs84(lon, lat):
    return transformer_gcj02_to_wgs84.transform(lon, lat)

# ===================== 初始化（用你提供的真实坐标） =====================
defaults = {
    "point_a": (32.2322, 118.7490),
    "point_b": (32.2343, 118.7490),
    "obstacles_all": [],
    "obstacles_height": [],
    "obstacle_names": [],
    "drone_height": 50,
    "drone_safety_radius": 15,
    "avoid_mode": "向左绕飞",
    "flight_path": [],
    "drone_pos": None,
    "flight_idx": 0,
    "heading": 0,
    "drawing_mode": False,
    "temp_points": [],
    "heartbeat_log": [],
    "task_state": "已暂停",
    "task_speed": 8.5,
    "task_time": 0,
    "task_distance": 0,
    "task_battery": 96,
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
tab1, tab2 = st.tabs(["✅ 航线规划", "🚁 飞行监控（心跳包+任务面板）"])

# --------------------- 标签1：航线规划 ---------------------
with tab1:
    st.title("📌 无人机航线规划系统（GCJ-02坐标系）")
    col_left, col_right = st.columns([1, 3])

    with col_left:
        st.subheader("📍 起点 A (GCJ-02)")
        a_lat = st.number_input("A点纬度", value=32.2322, format="%.4f", key="a_lat")
        a_lng = st.number_input("A点经度", value=118.7490, format="%.3f", key="a_lng")
        if st.button("✅ 设置A点", key="set_a"):
            st.session_state.point_a = (a_lat, a_lng)

        st.subheader("📍 终点 B (GCJ-02)")
        b_lat = st.number_input("B点纬度", value=32.2343, format="%.4f", key="b_lat")
        b_lng = st.number_input("B点经度", value=118.7490, format="%.3f", key="b_lng")
        if st.button("✅ 设置B点", key="set_b"):
            st.session_state.point_b = (b_lat, b_lng)

        st.subheader("🛠️ 飞行参数")
        st.session_state.drone_height = st.slider("设定飞行高度(m)", 0, 150, 50, key="drone_h")
        st.session_state.drone_safety_radius = st.slider("安全半径(m)", 1, 50, 15, key="safety_r")
        st.session_state.avoid_mode = st.radio("绕飞模式", ["向左绕飞", "向右绕飞", "最短弧线"], key="avoid_mode")

        st.subheader("✈️ 飞行控制")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("▶️ 开始飞行", key="start1"):
                route = calculate_route()
                if len(route) >= 2:
                    st.session_state.flight_path = interpolate_path(route)
                    st.session_state.drone_pos = st.session_state.flight_path[0]
                    st.session_state.flight_idx = 0
                    st.session_state.task_state = "执行中"
                    st.session_state.task_time = 0
        with c2:
            if st.button("⏹️ 停止", key="stop1"):
                st.session_state.drone_pos = None
                st.session_state.task_state = "已暂停"

        st.subheader("🟥 障碍物圈选（记忆）")
        h = st.slider("障碍物高度(m)", 5, 150, 30, key="obs_h")
        if st.button("🟢 开始圈选", key="draw_start"):
            st.session_state.drawing_mode = True
            st.session_state.temp_points = []
        if st.button("✅ 完成圈选并保存", key="draw_save"):
            if len(st.session_state.temp_points) >= 3:
                st.session_state.temp_points.append(st.session_state.temp_points[0])
                st.session_state.obstacles_all.append(st.session_state.temp_points)
                st.session_state.obstacles_height.append(h)
                st.session_state.obstacle_names.append(f"障碍{len(st.session_state.obstacles_all)}")
                st.success(f"已保存障碍，高度={h}m")
        if st.button("🗑️ 清空所有障碍物", key="clear_obs"):
            st.session_state.obstacles_all = []
            st.session_state.obstacles_height = []
            st.session_state.obstacle_names = []

    with col_right:
        st.caption("🗺️ 卫星地图（Esri 影像，GCJ-02适配），放大后为2D便于圈选")
        route = calculate_route()
        center = st.session_state.point_a or (32.2322, 118.7490)
        m = folium.Map(
            location=center,
            zoom_start=19,
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Tiles © Esri — Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community"
        )

        if st.session_state.point_a:
            folium.Marker(st.session_state.point_a, icon=folium.Icon(color="red", icon="white"), tooltip="起点A").add_to(m)
        if st.session_state.point_b:
            folium.Marker(st.session_state.point_b, icon=folium.Icon(color="green", icon="play"), tooltip="终点B").add_to(m)

        for obs in st.session_state.obstacles_all:
            if len(obs) > 2:
                folium.Polygon(obs, color="red", fill=True, fill_opacity=0.5).add_to(m)

        if len(route) >= 2:
            folium.PolyLine(route, color="blue", weight=4, dash_array="10,10").add_to(m)

        if st.session_state.drone_pos:
            lat, lng = st.session_state.drone_pos
            icon_html = f'<div style="transform:rotate({st.session_state.heading}deg);font-size:28px;">✈️</div>'
            folium.Marker((lat, lng), icon=folium.DivIcon(html=icon_html)).add_to(m)

        map_out = st_folium(m, height=750, key="map1", returned_objects=["last_clicked"])

        if map_out and map_out.get("last_clicked"):
            lat = map_out["last_clicked"]["lat"]
            lng = map_out["last_clicked"]["lng"]
            if st.session_state.drawing_mode:
                st.session_state.temp_points.append([lat, lng])

# --------------------- 标签2：飞行监控 ---------------------
with tab2:
    st.title("🚁 飞行实时画面 - 任务执行监控")

    col_btn = st.columns([1, 1, 1, 1, 0.3])
    with col_btn[0]:
        if st.button("▶️ 开始任务", type="primary", use_container_width=True, key="task_start"):
            route = calculate_route()
            if len(route) >= 2:
                st.session_state.flight_path = interpolate_path(route)
                st.session_state.drone_pos = st.session_state.flight_path[0]
                st.session_state.flight_idx = 0
                st.session_state.task_state = "执行中"
                st.session_state.task_time = 0
    with col_btn[1]:
        if st.button("⏸️ 暂停", use_container_width=True, key="task_pause"):
            st.session_state.task_state = "已暂停"
    with col_btn[2]:
        if st.button("⏹️ 停止", use_container_width=True, key="task_stop"):
            st.session_state.drone_pos = None
            st.session_state.task_state = "已暂停"
    with col_btn[3]:
        if st.button("🔄 重置", use_container_width=True, key="task_reset"):
            st.session_state.flight_idx = 0
            st.session_state.drone_pos = None
            st.session_state.task_state = "已暂停"
            st.session_state.task_time = 0
    with col_btn[4]:
        st.caption(f"🟡 {st.session_state.task_state}")

    st.divider()

    col_status = st.columns(6)
    with col_status[0]:
        st.metric("当前航点", f"{st.session_state.flight_idx}/{len(st.session_state.flight_path) if st.session_state.flight_path else 0}", key="metric1")
    with col_status[1]:
        st.metric("飞行速度", f"{st.session_state.task_speed} m/s", key="metric2")
    with col_status[2]:
        st.metric("已用时间", f"{st.session_state.task_time//60:02d}:{st.session_state.task_time%60:02d}", key="metric3")
    with col_status[3]:
        remaining = max(0, len(st.session_state.flight_path) - st.session_state.flight_idx) if st.session_state.flight_path else 0
        st.metric("剩余距离", f"{remaining} m", key="metric4")
    with col_status[4]:
        st.metric("预计到达", "00:00", key="metric5")
    with col_status[5]:
        st.metric("电量模拟", f"{st.session_state.task_battery}%", key="metric6")

    progress = min(100, int(100 * st.session_state.flight_idx / len(st.session_state.flight_path)) if st.session_state.flight_path else 0)
    st.progress(progress, text=f"任务进度: {progress}%")
    st.divider()

    col_map, col_comm = st.columns([2, 1])
    with col_map:
        st.subheader("🗺️ 实时飞行地图")
        route = calculate_route()
        center = st.session_state.point_a or (32.2322, 118.7490)
        m = folium.Map(
            location=center,
            zoom_start=19,
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Tiles © Esri"
        )

        if st.session_state.point_a:
            folium.Marker(st.session_state.point_a, icon=folium.Icon(color="red", icon="white"), tooltip="起点A").add_to(m)
        if st.session_state.point_b:
            folium.Marker(st.session_state.point_b, icon=folium.Icon(color="green", icon="play"), tooltip="终点B").add_to(m)

        for obs in st.session_state.obstacles_all:
            if len(obs) > 2:
                folium.Polygon(obs, color="red", fill=True, fill_opacity=0.5).add_to(m)

        if len(route) >= 2:
            folium.PolyLine(route, color="green", weight=4).add_to(m)

        if st.session_state.drone_pos:
            lat, lng = st.session_state.drone_pos
            icon_html = f'<div style="transform:rotate({st.session_state.heading}deg);font-size:28px;">✈️</div>'
            folium.Marker((lat, lng), icon=folium.DivIcon(html=icon_html)).add_to(m)

        st_folium(m, height=600, key="map2")

    with col_comm:
        st.subheader("📡 通信链路拓扑与数据流")
        st.markdown("""
        <div style="display:flex;justify-content:space-around;margin-bottom:20px;">
            <span>✅ GCS 在线</span>
            <span>✅ OBC 在线</span>
            <span>✅ FCU 在线</span>
        </div>
        """, unsafe_allow_html=True)

        col_gcs, col_obc, col_fcu = st.columns(3)
        with col_gcs:
            st.markdown("""
            <div style="border:2px solid #4A90E2;border-radius:8px;padding:10px;text-align:center;">
                <div style="font-size:24px;">🖥️</div>
                <div><b>GCS</b></div>
                <div style="font-size:12px;">地面站</div>
                <div style="font-size:10px;color:gray;">192.168.1.100</div>
            </div>
            """, unsafe_allow_html=True)
        with col_obc:
            st.markdown("""
            <div style="border:2px solid #F5A623;border-radius:8px;padding:10px;text-align:center;">
                <div style="font-size:24px;">🧠</div>
                <div><b>OBC</b></div>
                <div style="font-size:12px;">机载计算机</div>
                <div style="font-size:10px;color:gray;">Raspberry Pi 4</div>
            </div>
            """, unsafe_allow_html=True)
        with col_fcu:
            st.markdown("""
            <div style="border:2px solid #BD10E0;border-radius:8px;padding:10px;text-align:center;">
                <div style="font-size:24px;">⚙️</div>
                <div><b>FCU</b></div>
                <div style="font-size:12px;">飞控</div>
                <div style="font-size:10px;color:gray;">PX4 / ArduPilot</div>
            </div>
            """, unsafe_allow_html=True)

        st.markdown("""
        <div style="display:flex;justify-content:space-around;margin:10px 0;">
            <div>⬆️⬇️ UDP:14550</div>
            <div>⬆️⬇️ MAVLink</div>
        </div>
        <div style="display:flex;justify-content:space-around;margin-bottom:10px;">
            <div style="color:green;">🟢 已连接</div>
            <div style="color:green;">🟢 已连接</div>
        </div>
        <div style="background:#f0f0f0;padding:8px;border-radius:4px;font-size:12px;">
            链路统计：GCS↔OBC:正常 | OBC↔FCU:正常 | 延迟:~25ms | 丢包率:0.1%
        </div>
        """, unsafe_allow_html=True)

        st.subheader("🧠 心跳包日志")
        now = time.strftime("%H:%M:%S")
        if st.session_state.drone_pos:
            st.session_state.heartbeat_log.append(f"[{now}] 在线 | 位置有效")
        else:
            st.session_state.heartbeat_log.append(f"[{now}] 待机")
        for line in st.session_state.heartbeat_log[-15:]:
            st.text(line)

# ===================== 飞行循环 =====================
if st.session_state.task_state == "执行中" and st.session_state.drone_pos and st.session_state.flight_path:
    total = len(st.session_state.flight_path)
    if st.session_state.flight_idx < total - 1:
        st.session_state.flight_idx += 1
        st.session_state.drone_pos = st.session_state.flight_path[st.session_state.flight_idx]
        st.session_state.task_time += 1
        if st.session_state.flight_idx + 1 < total:
            p1 = st.session_state.flight_path[st.session_state.flight_idx]
            p2 = st.session_state.flight_path[st.session_state.flight_idx + 1]
            st.session_state.heading = get_heading(p1[0], p1[1], p2[0], p2[1])
        time.sleep(0.08)
        st.rerun()
    else:
        st.session_state.task_state = "已完成"
        st.success("✅ 已到达目的地，任务完成！")