import streamlit as st
import folium
from streamlit_folium import st_folium
import pandas as pd
import time
from datetime import datetime

# ================== 初始化 ==================
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

st.set_page_config(page_title="无人机航线", layout="wide")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("选择页面", ["航线规划", "飞行监控"])

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

# ================== 航线规划 ==================
if page == "航线规划":
    st.title("🗺️ 卫星地图航线规划")
    st.success("👉 点击地图 → 侧边栏设置A/B点，点位百分百精准")

    # 固定中心，不跳动
    center_lat = 32.2330
    center_lng = 118.7490

    # 地图
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="卫星地图"
    )

    # ========== A 点：绿色 + 大字 A 起点 ==========
    if st.session_state.point_a:
        lat_a, lng_a = st.session_state.point_a
        folium.CircleMarker(
            location=(lat_a, lng_a), radius=8, color='green', fill=True, fill_color='green'
        ).add_to(m)
        folium.Marker(
            location=(lat_a, lng_a),
            icon=folium.DivIcon(
                html='<div style="color:white; font-weight:bold; font-size:14px;">A 起点</div>'
            )
        ).add_to(m)

    # ========== B 点：红色 + 大字 B 终点 ==========
    if st.session_state.point_b:
        lat_b, lng_b = st.session_state.point_b
        folium.CircleMarker(
            location=(lat_b, lng_b), radius=8, color='red', fill=True, fill_color='red'
        ).add_to(m)
        folium.Marker(
            location=(lat_b, lng_b),
            icon=folium.DivIcon(
                html='<div style="color:white; font-weight:bold; font-size:14px;">B 终点</div>'
            )
        ).add_to(m)

    # 航线
    if st.session_state.point_a and st.session_state.point_b:
        folium.PolyLine(
            locations=[st.session_state.point_a, st.session_state.point_b],
            color='blue', weight=4
        ).add_to(m)

    # 稳定渲染，只接收点击，不刷新页面
    map_out = st_folium(
        m, key="drones_map", height=650,
        use_container_width=True,
        returned_objects=["last_clicked"]
    )

    # 百分百精准捕获坐标
    if map_out and map_out.get("last_clicked"):
        clat = map_out["last_clicked"]["lat"]
        clng = map_out["last_clicked"]["lng"]
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