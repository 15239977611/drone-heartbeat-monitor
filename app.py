import streamlit as st
import folium
from streamlit_folium import st_folium
import pandas as pd
import json
from datetime import datetime

# ================== 初始化 ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "drawing" not in st.session_state:
    st.session_state.drawing = False

# ================== 本地存储 ==================
def save_obs():
    with open("obstacles.json", "w", encoding="utf-8") as f:
        json.dump(st.session_state.obstacles, f)

def load_obs():
    try:
        with open("obstacles.json", "r", encoding="utf-8") as f:
            st.session_state.obstacles = json.load(f)
    except:
        st.session_state.obstacles = []

load_obs()
st.set_page_config(page_title="无人机航线", layout="wide")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("🚧 障碍物（多边形圈选）")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始绘制"):
            st.session_state.drawing = True
    with col2:
        if st.button("✅ 结束绘制"):
            st.session_state.drawing = False
    if st.button("🗑️ 清空障碍物"):
        st.session_state.obstacles = []
        save_obs()

    st.markdown("---")
    st.subheader("🟢 A 点（起点）")
    if st.session_state.point_a:
        st.success(f"纬度：{st.session_state.point_a[0]:.6f}")
        st.success(f"经度：{st.session_state.point_a[1]:.6f}")
        if st.button("🗑️ 清除A点"):
            st.session_state.point_a = None
    else:
        st.warning("未设置")

    st.markdown("---")
    st.subheader("🔴 B 点（终点）")
    if st.session_state.point_b:
        st.success(f"纬度：{st.session_state.point_b[0]:.6f}")
        st.success(f"经度：{st.session_state.point_b[1]:.6f}")
        if st.button("🗑️ 清除B点"):
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

    # 障碍物
    for obs in st.session_state.obstacles:
        if len(obs) > 2:
            folium.Polygon(locations=obs, color='red', fill=True, fill_color='red', fill_opacity=0.3, weight=3).add_to(m)

    # ✅ 关键：不刷新、不闪烁、前端绘制
    map_data = st_folium(
        m, key="map", height=650,
        use_container_width=True,
        returned_objects=[] if st.session_state.drawing else ["last_clicked"]
    )

    # 只在不绘制时接收A/B点点击
    if not st.session_state.drawing and map_data and map_data.get("last_clicked"):
        lat = map_data["last_clicked"]["lat"]
        lng = map_data["last_clicked"]["lng"]
        st.session_state.point_a = (lat, lng)

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    if st.button("▶️ 开始 / ⏹️ 停止"):
        st.session_state.sim = not st.session_state.get("sim", False)
    if st.session_state.get("sim", False):
        st.success("✅ 在线")
        st.session_state.data = pd.concat([st.session_state.data, pd.DataFrame([{"t":datetime.now(),"v":1}])], ignore_index=True)
        st.line_chart(st.session_state.data.set_index("t")["v"])
    else:
        st.info("已停止")