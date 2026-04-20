import streamlit as st
import folium
from streamlit_folium import st_folium
import pandas as pd
from datetime import datetime
import time
import math

# ================== 坐标系转换（GCJ-02 → WGS-84）==================
def gcj02_to_wgs84(lat, lng):
    a = 6378245.0
    ee = 0.00669342162296594323

    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y**2 + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
        return ret

    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x**2 + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
        return ret

    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    return lat - dlat, lng - dlng

# ================== 初始化 Session ==================
def init_state():
    defaults = {
        "point_a": None,
        "point_b": None,
        "last_clicked": None,
        "obstacles": [],
        "heartbeat_data": pd.DataFrame(columns=["序号", "时间"]),
        "simulation_on": False,
        "heartbeat_seq": 0,
        "is_connected": True,
        "coord_type": "GCJ-02"
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v

init_state()

# ================== 心跳 ==================
def add_heartbeat():
    st.session_state.heartbeat_seq += 1
    new_row = pd.DataFrame([{
        "序号": st.session_state.heartbeat_seq,
        "时间": datetime.now()
    }])
    st.session_state.heartbeat_data = pd.concat(
        [st.session_state.heartbeat_data, new_row],
        ignore_index=True
    )
    st.session_state.last_hb = datetime.now()
    st.session_state.is_connected = True

def check_connection():
    if (datetime.now() - st.session_state.last_hb).total_seconds() > 3:
        st.session_state.is_connected = False

# ================== 页面 ==================
st.set_page_config(page_title="无人机监测", layout="wide")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("坐标系")
    coord_type = st.radio(
        "选择",
        ["GCJ-02(高德/百度)", "WGS-84"],
        index=0 if st.session_state.coord_type == "GCJ-02" else 1
    )
    st.session_state.coord_type = "GCJ-02" if "GCJ" in coord_type else "WGS-84"

    st.markdown("---")
    # 点击地图后出现选点
    if st.session_state.last_clicked:
        lat, lng = st.session_state.last_clicked
        st.info(f"📍 点击位置\n{lat:.6f}, {lng:.6f}")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("✅ 设为A点"):
                st.session_state.point_a = [lat, lng]
                st.session_state.last_clicked = None
                st.rerun()
        with c2:
            if st.button("✅ 设为B点"):
                st.session_state.point_b = [lat, lng]
                st.session_state.last_clicked = None
                st.rerun()
        if st.button("❌ 取消"):
            st.session_state.last_clicked = None
            st.rerun()

    st.markdown("---")
    st.subheader("A点（起点）")
    if st.session_state.point_a:
        st.success(f"纬度：{st.session_state.point_a[0]:.6f}")
        st.success(f"经度：{st.session_state.point_a[1]:.6f}")
        if st.button("🗑️ 清除A点"):
            st.session_state.point_a = None
            st.rerun()
    else:
        st.warning("未设置")

    st.markdown("---")
    st.subheader("B点（终点）")
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
    st.success("👉 点击地图任意位置 → 侧边栏选择设为A/B点")

    # 地图中心
    center = [32.2330, 118.7490]
    if st.session_state.point_a:
        center = st.session_state.point_a
    elif st.session_state.point_b:
        center = st.session_state.point_b

    # 建地图
    m = folium.Map(
        location=center,
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="卫星地图"
    )

    # 坐标转换
    def convert(pt):
        if not pt:
            return None
        lat, lng = pt
        if st.session_state.coord_type == "GCJ-02":
            return gcj02_to_wgs84(lat, lng)
        return lat, lng

    a = convert(st.session_state.point_a)
    b = convert(st.session_state.point_b)

    # A点（绿）
    if a:
        folium.Marker(
            location=a,
            icon=folium.Icon(color="green", icon="play", prefix="fa"),
            popup="A 起点"
        ).add_to(m)

    # B点（红）
    if b:
        folium.Marker(
            location=b,
            icon=folium.Icon(color="red", icon="flag", prefix="fa"),
            popup="B 终点"
        ).add_to(m)

    # 航线
    if a and b:
        folium.PolyLine(
            locations=[a, b],
            color="blue",
            weight=4
        ).add_to(m)

    # 渲染地图（关键：接收点击）
    map_data = st_folium(m, key="map", height=600, width="100%")

    # 接收点击
    if map_data and map_data.get("last_clicked"):
        lat = map_data["last_clicked"]["lat"]
        lng = map_data["last_clicked"]["lng"]
        st.session_state.last_clicked = [lat, lng]
        st.rerun()

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
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("✅ 在线")
    else:
        st.error("🚨 断开")

    if not st.session_state.heartbeat_data.empty:
        st.line_chart(
            st.session_state.heartbeat_data.tail(50).set_index("时间")["序号"]
        )