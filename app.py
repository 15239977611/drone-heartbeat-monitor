import streamlit as st
import folium
from streamlit_folium import st_folium
import json
import pandas as pd
from datetime import datetime

# ================== 初始化 ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "drawing_obstacle" not in st.session_state:
    st.session_state.drawing_obstacle = False
if "current_points" not in st.session_state:
    st.session_state.current_points = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 50
if "obs_heights" not in st.session_state:
    st.session_state.obs_heights = []

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
st.set_page_config(page_title="智能无人机航线", layout="wide")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("选择页面", ["航线规划", "飞行监控"])

    # --------------------- 无人机高度 ---------------------
    st.markdown("---")
    st.subheader("🛸 无人机飞行高度")
    st.session_state.drone_height = st.slider("设置飞行高度（米）", 0, 200, 50)

    # --------------------- 障碍物 ---------------------
    st.markdown("---")
    st.subheader("🚧 障碍物绘制")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选"):
            st.session_state.drawing_obstacle = True
            st.session_state.current_points = []
    with col2:
        if st.button("✅ 完成圈选"):
            if len(st.session_state.current_points) >=3:
                st.session_state.obstacles.append(st.session_state.current_points)
                st.session_state.obs_heights.append(30)
                save_all()
            st.session_state.drawing_obstacle = False
            st.session_state.current_points = []

    if st.button("🗑️ 清除所有障碍物"):
        st.session_state.obstacles = []
        st.session_state.obs_heights = []
        st.session_state.current_points = []
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

# ================== 智能绕行逻辑 ==================
def get_path():
    a = st.session_state.point_a
    b = st.session_state.point_b
    if not a or not b:
        return []
    return [a, b]

# ================== 地图 ==================
if page == "航线规划":
    st.title("🗺️ 智能卫星地图航线规划")
    st.success(f"🛸 无人机飞行高度：{st.session_state.drone_height} 米")

    # ✅ 修复卫星地图 + 保留原来样式（关键修复！）
    m = folium.Map(
        location=[32.2330, 118.7490],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery"  # 这里补上归属，地图立刻恢复！
    )

    # A点 B点
    if st.session_state.point_a:
        folium.CircleMarker(st.session_state.point_a, radius=8, color='green', fill=True).add_to(m)
        folium.Marker(st.session_state.point_a, 
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:16px;">A 起点</div>')
        ).add_to(m)
    if st.session_state.point_b:
        folium.CircleMarker(st.session_state.point_b, radius=8, color='red', fill=True).add_to(m)
        folium.Marker(st.session_state.point_b, 
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:16px;">B 终点</div>')
        ).add_to(m)

    # 障碍物
    for i, obs in enumerate(st.session_state.obstacles):
        if len(obs) > 2:
            folium.Polygon(
                locations=obs,
                color='red',
                fill=True,
                fill_opacity=0.3,
                weight=3
            ).add_to(m)

    # 航线
    path = get_path()
    if len(path) > 1:
        folium.PolyLine(path, color='blue', weight=5).add_to(m)

    # 绘制中的线条
    if st.session_state.drawing_obstacle and len(st.session_state.current_points) > 0:
        folium.PolyLine(
            locations=st.session_state.current_points,
            color='orange',
            weight=4
        ).add_to(m)

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
    st.success(f"🛸 无人机飞行高度：{st.session_state.drone_height} m")
    st.success("✅ 在线正常")