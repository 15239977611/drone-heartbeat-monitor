import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point
from datetime import datetime

# ================== 初始化 ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "obs_heights" not in st.session_state:
    st.session_state.obs_heights = []
if "drawing_obstacle" not in st.session_state:
    st.session_state.drawing_obstacle = False
if "current_points" not in st.session_state:
    st.session_state.current_points = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 50

# ================== 永久存储 ==================
def save_all():
    with open("obstacles.json", "w", encoding="utf-8") as f:
        json.dump({
            "obs": st.session_state.obstacles,
            "heights": st.session_state.obs_heights
        }, f)

def load_all():
    try:
        with open("obstacles.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles = data.get("obs", [])
            st.session_state.obs_heights = data.get("heights", [])
    except:
        st.session_state.obstacles = []
        st.session_state.obs_heights = []

load_all()
st.set_page_config(page_title="无人机智能绕行系统", layout="wide")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("选择页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("🛸 无人机飞行高度")
    st.session_state.drone_height = st.slider("飞行高度（米）", 0, 200, 50)

    st.markdown("---")
    st.subheader("🚧 实体障碍物（禁区）")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选禁区"):
            st.session_state.drawing_obstacle = True
            st.session_state.current_points = []
    with col2:
        if st.button("✅ 完成圈选"):
            if len(st.session_state.current_points) >= 3:
                st.session_state.obstacles.append(st.session_state.current_points)
                # 让用户设置障碍物高度
                height = st.number_input("设置障碍物高度", 1, 200, 30)
                st.session_state.obs_heights.append(height)
                save_all()
            st.session_state.drawing_obstacle = False
            st.session_state.current_points = []

    if st.button("🗑️ 清除所有障碍物"):
        st.session_state.obstacles = []
        st.session_state.obs_heights = []
        save_all()

    st.markdown("---")
    st.subheader("🟢 A 点（起点）")
    if st.session_state.point_a:
        st.success(f"纬度：{st.session_state.point_a[0]:.6f}")
        st.success(f"经度：{st.session_state.point_a[1]:.6f}")
        if st.button("🗑️ 清除 A 点"):
            st.session_state.point_a = None
    else:
        st.warning("未设置")

    st.markdown("---")
    st.subheader("🔴 B 点（终点）")
    if st.session_state.point_b:
        st.success(f"纬度：{st.session_state.point_b[0]:.6f}")
        st.success(f"经度：{st.session_state.point_b[1]:.6f}")
        if st.button("🗑️ 清除 B 点"):
            st.session_state.point_b = None
    else:
        st.warning("未设置")

# ================== ✅ 核心：最佳绕行算法 ==================
def best_path():
    A = st.session_state.point_a
    B = st.session_state.point_b
    if not A or not B:
        return []

    waypoints = [A]
    current = A
    path_line = LineString([A, B])

    for obs, h in zip(st.session_state.obstacles, st.session_state.obs_heights):
        try:
            poly = Polygon(obs)
            if poly.intersects(path_line):
                # 计算障碍物外侧最近点，实现平滑绕行
                centroid = poly.centroid
                dx = centroid.x - current[0]
                dy = centroid.y - current[1]
                # 偏移一小段距离，绕开禁区
                offset = 0.00025
                side_point = (
                    centroid.x + (-dy if dy > 0 else dy) * offset,
                    centroid.y + (dx if dx > 0 else -dx) * offset
                )
                waypoints.append(side_point)
        except:
            continue

    waypoints.append(B)
    return waypoints

# ================== 地图 ==================
if page == "航线规划":
    st.title("🗺️ 无人机智能绕行系统")
    st.success("✅ 圈出禁区 → 自动规划最佳绕行路线")

    m = folium.Map(
        location=[32.2330, 118.7490],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery"
    )

    # A B 点
    if st.session_state.point_a:
        folium.CircleMarker(st.session_state.point_a, radius=9, color='green', fill=True).add_to(m)
        folium.Marker(st.session_state.point_a, icon=folium.DivIcon(html='<div style="color:white; font-weight:bold;">A 起点</div>')).add_to(m)
    if st.session_state.point_b:
        folium.CircleMarker(st.session_state.point_b, radius=9, color='red', fill=True).add_to(m)
        folium.Marker(st.session_state.point_b, icon=folium.DivIcon(html='<div style="color:white; font-weight:bold;">B 终点</div>')).add_to(m)

    # 绘制障碍物（红色禁区）
    for i, obs in enumerate(st.session_state.obstacles):
        if len(obs) > 2:
            h = st.session_state.obs_heights[i]
            folium.Polygon(
                locations=obs,
                color='red',
                fill=True,
                fill_color='red',
                fill_opacity=0.4,
                weight=3,
                popup=f"障碍物高度：{h}m"
            ).add_to(m)

    # ✅ 最佳路线
    path = best_path()
    if len(path) > 1:
        folium.PolyLine(
            locations=path,
            color='blue',
            weight=5,
            opacity=0.8
        ).add_to(m)

    # 绘制中的线条
    if st.session_state.drawing_obstacle and len(st.session_state.current_points) > 0:
        folium.PolyLine(st.session_state.current_points, color='orange', weight=4).add_to(m)

    # 地图点击
    map_out = st_folium(m, key="map", height=650, use_container_width=True, returned_objects=["last_clicked"])
    if map_out and map_out.get("last_clicked"):
        lat = round(map_out["last_clicked"]["lat"], 6)
        lng = round(map_out["last_clicked"]["lng"], 6)
        if st.session_state.drawing_obstacle:
            st.session_state.current_points.append([lat, lng])
        else:
            if not st.session_state.point_a:
                st.session_state.point_a = (lat, lng)
            else:
                st.session_state.point_b = (lat, lng)

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    st.success(f"🛸 飞行高度：{st.session_state.drone_height} m")
    st.success("✅ 智能绕行系统已启动")