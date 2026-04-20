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

# ================== 永久存储 ==================
def save_obstacles():
    with open("obstacles.json", "w", encoding="utf-8") as f:
        json.dump(st.session_state.obstacles, f)

def load_obstacles():
    try:
        with open("obstacles.json", "r", encoding="utf-8") as f:
            st.session_state.obstacles = json.load(f)
    except:
        st.session_state.obstacles = []

load_obstacles()
st.set_page_config(page_title="无人机航线", layout="wide")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("选择页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("🚧 障碍物多边形绘制")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选障碍物"):
            st.session_state.drawing_obstacle = True
            st.session_state.current_points = []
    with col2:
        if st.button("✅ 完成圈选"):
            if len(st.session_state.current_points) >= 3:
                st.session_state.obstacles.append(st.session_state.current_points)
                save_obstacles()
            st.session_state.drawing_obstacle = False
            st.session_state.current_points = []

    if st.button("🗑️ 清除所有障碍物"):
        st.session_state.obstacles = []
        st.session_state.current_points = []
        save_obstacles()

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

# ================== 地图 ==================
if page == "航线规划":
    st.title("🗺️ 卫星地图航线规划")

    m = folium.Map(
        location=[32.2330, 118.7490],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="卫星地图"
    )

    # A点
    if st.session_state.point_a:
        folium.CircleMarker(location=st.session_state.point_a, radius=8, color='green', fill=True).add_to(m)
        folium.Marker(location=st.session_state.point_a, icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:16px;">A 起点</div>')).add_to(m)
    # B点
    if st.session_state.point_b:
        folium.CircleMarker(location=st.session_state.point_b, radius=8, color='red', fill=True).add_to(m)
        folium.Marker(location=st.session_state.point_b, icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:16px;">B 终点</div>')).add_to(m)
    # 航线
    if st.session_state.point_a and st.session_state.point_b:
        folium.PolyLine([st.session_state.point_a, st.session_state.point_b], color='blue', weight=4).add_to(m)

    # 已保存障碍物
    for obs in st.session_state.obstacles:
        if len(obs) > 2:
            folium.Polygon(locations=obs, color='red', fill=True, fill_opacity=0.3, weight=3).add_to(m)

    # 正在绘制的线
    if st.session_state.drawing_obstacle and len(st.session_state.current_points) > 0:
        folium.PolyLine(
            locations=st.session_state.current_points,
            color='orange', weight=4
        ).add_to(m)
        for p in st.session_state.current_points:
            folium.CircleMarker(location=p, radius=4, color='orange', fill=True).add_to(m)

    # ================== ✅ 核心修复：不闪烁 + 能正常绘制 ==================
    map_out = st_folium(
        m, key="drones_map", height=650,
        use_container_width=True,
        returned_objects=["last_clicked"]
    )

    # 接收点击
    if map_out and map_out.get("last_clicked"):
        lat = round(map_out["last_clicked"]["lat"], 6)
        lng = round(map_out["last_clicked"]["lng"], 6)

        if st.session_state.drawing_obstacle:
            st.session_state.current_points.append([lat, lng])
        else:
            st.session_state.point_a = (lat, lng)

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    if st.button("▶️ 开始 / ⏹️ 停止"):
        st.session_state.sim = not st.session_state.get("sim", False)

    if st.session_state.get("sim", False):
        st.success("✅ 在线")
        df = pd.DataFrame({
            "时间": pd.date_range(end=datetime.now(), periods=20, freq="1s"),
            "信号": [1]*20
        })
        st.line_chart(df.set_index("时间"))
    else:
        st.info("已停止")