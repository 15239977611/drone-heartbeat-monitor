import streamlit as st
import folium
from streamlit_folium import st_folium
import pandas as pd
import time
import math
from datetime import datetime

# ================== 坐标系转换 ==================
def gcj02_to_wgs84(lat, lng):
    a = 6378245.0
    ee = 0.00669342162296594323

    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y**2 + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320.0 * math.sin(y / 30.0 * math.pi)) / 3.0
        return ret

    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x**2 + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) / 2.0
        ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) / 3.0
        ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) / 6.0
        return ret

    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a * sqrtmagic * math.cos(radlat) * math.pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lat * 2 - mglat, lng * 2 - mglng

# ================== 初始化（防报错） ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "last_clicked" not in st.session_state:
    st.session_state.last_clicked = None
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = pd.DataFrame(columns=["序号", "时间"])
if "simulation_on" not in st.session_state:
    st.session_state.simulation_on = False
if "coord_type" not in st.session_state:
    st.session_state.coord_type = "GCJ-02"

st.set_page_config(page_title="无人机航线", layout="wide")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("选择页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("坐标系")
    coord_opt = st.radio("", ["GCJ-02(高德/百度)", "WGS-84"])
    st.session_state.coord_type = "GCJ-02" if "GCJ" in coord_opt else "WGS-84"

    st.markdown("---")
    # 点击后选择A/B点
    if st.session_state.last_clicked is not None:
        lat, lng = st.session_state.last_clicked
        st.info(f"📍 选中位置\n{lat:.6f}, {lng:.6f}")
        
        col1, col2 = st.columns(2)
        with col1:
            if st.button("✅ 设为A点"):
                st.session_state.point_a = (lat, lng)
                st.session_state.last_clicked = None
                st.rerun()
        with col2:
            if st.button("✅ 设为B点"):
                st.session_state.point_b = (lat, lng)
                st.session_state.last_clicked = None
                st.rerun()
        
        if st.button("❌ 取消选择"):
            st.session_state.last_clicked = None
            st.rerun()

    st.markdown("---")
    st.subheader("A 点（起点）")
    if st.session_state.point_a:
        st.success(f"纬度：{st.session_state.point_a[0]:.6f}")
        st.success(f"经度：{st.session_state.point_a[1]:.6f}")
        if st.button("🗑️ 清除A点"):
            st.session_state.point_a = None
            st.rerun()
    else:
        st.warning("未设置")

    st.markdown("---")
    st.subheader("B 点（终点）")
    if st.session_state.point_b:
        st.success(f"纬度：{st.session_state.point_b[0]:.6f}")
        st.success(f"经度：{st.session_state.point_b[1]:.6f}")
        if st.button("🗑️ 清除B点"):
            st.session_state.point_b = None
            st.rerun()
    else:
        st.warning("未设置")

# ================== 航线规划（稳定不闪烁） ==================
if page == "航线规划":
    st.title("🗺️ 卫星地图航线规划")
    st.success("👉 点击地图 → 侧边栏设置A/B点")

    # 地图中心（不自动跳转）
    center_lat = 32.2330
    center_lng = 118.7490
    if st.session_state.point_a:
        center_lat, center_lng = st.session_state.point_a
    elif st.session_state.point_b:
        center_lat, center_lng = st.session_state.point_b

    # 创建地图（禁用自动刷新 → 不闪烁）
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="卫星地图",
        control_scale=True
    )

    # 坐标转换（精准）
    def get_pos(pt):
        if not pt:
            return None
        lat, lng = pt
        if st.session_state.coord_type == "GCJ-02":
            return gcj02_to_wgs84(lat, lng)
        return (lat, lng)

    a = get_pos(st.session_state.point_a)
    b = get_pos(st.session_state.point_b)

    # A点：绿色起点
    if a:
        folium.CircleMarker(
            location=a, radius=10, color="green", fill=True, fill_color="green", fill_opacity=1
        ).add_to(m)
        folium.Marker(
            location=a,
            icon=folium.DivIcon(html='<div style="color:white; background:green; padding:2px 6px; border-radius:6px; font-weight:bold;">A 起点</div>')
        ).add_to(m)

    # B点：红色终点
    if b:
        folium.CircleMarker(
            location=b, radius=10, color="red", fill=True, fill_color="red", fill_opacity=1
        ).add_to(m)
        folium.Marker(
            location=b,
            icon=folium.DivIcon(html='<div style="color:white; background:red; padding:2px 6px; border-radius:6px; font-weight:bold;">B 终点</div>')
        ).add_to(m)

    # 航线
    if a and b:
        folium.PolyLine([a, b], color="blue", weight=4).add_to(m)

    # 渲染地图（关键：不刷新 → 不闪烁）
    output = st_folium(m, key="drones_map", height=650, use_container_width=True, returned_objects=["last_clicked"])

    # 接收点击（精准 + 不抖动）
    if output and output.get("last_clicked"):
        clat = round(output["last_clicked"]["lat"], 6)
        clng = round(output["last_clicked"]["lng"], 6)
        st.session_state.last_clicked = (clat, clng)

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳"):
            st.session_state.simulation_on = True
    with c2:
        if st.button("⏹️ 停止心跳"):
            st.session_state.simulation_on = False

    if st.session_state.simulation_on:
        st.session_state.heartbeat_data = pd.concat([
            st.session_state.heartbeat_data,
            pd.DataFrame([{"序号": len(st.session_state.heartbeat_data)+1, "时间": datetime.now()}])
        ])
        time.sleep(1)
        st.rerun()

    st.success("✅ 在线")
    if not st.session_state.heartbeat_data.empty:
        st.line_chart(st.session_state.heartbeat_data.set_index("时间")["序号"])