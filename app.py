import streamlit as st
import pandas as pd
import time
import math
import random
from datetime import datetime
from streamlit.components.v1 import html

# ================== 坐标系转换（GCJ-02 -> WGS-84） ==================
def gcj02_to_wgs84(lat, lng):
    a = 6378245.0
    ee = 0.00669342162296594323

    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
        return ret

    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
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
    wgs_lat = lat - dlat
    wgs_lng = lng - dlng
    return wgs_lat, wgs_lng

# ================== 心跳数据存储 ==================
if 'heartbeat_data' not in st.session_state:
    st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
if 'last_heartbeat_time' not in st.session_state:
    st.session_state.last_heartbeat_time = datetime.now()
if 'is_connected' not in st.session_state:
    st.session_state.is_connected = True
if 'heartbeat_sequence' not in st.session_state:
    st.session_state.heartbeat_sequence = 0
if 'simulation_on' not in st.session_state:
    st.session_state.simulation_on = False

# 障碍物
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []

def add_heartbeat():
    st.session_state.heartbeat_sequence += 1
    new_row = pd.DataFrame([{'序号': st.session_state.heartbeat_sequence, '时间': datetime.now()}])
    st.session_state.heartbeat_data = pd.concat([st.session_state.heartbeat_data, new_row], ignore_index=True)
    st.session_state.last_heartbeat_time = datetime.now()
    st.session_state.is_connected = True

def check_connection():
    now = datetime.now()
    if (now - st.session_state.last_heartbeat_time).total_seconds() > 3:
        st.session_state.is_connected = False

# ================== 页面配置 ==================
st.set_page_config(page_title="无人机智能监测系统", layout="wide")
st.sidebar.title("导航")
page = st.sidebar.radio("功能页面", ["航线规划", "飞行监控"])

# ================== 航线规划 ==================
if page == "航线规划":
    st.title("🗺️ 航线规划")
    default_a = (32.2322, 118.7490)
    default_b = (32.2343, 118.7490)

    if 'a_point' in st.session_state:
        a_lat_raw, a_lng_raw, a_sys = st.session_state.a_point
    else:
        a_lat_raw, a_lng_raw, a_sys = default_a[0], default_a[1], "GCJ-02"
    if 'b_point' in st.session_state:
        b_lat_raw, b_lng_raw, b_sys = st.session_state.b_point
    else:
        b_lat_raw, b_lng_raw, b_sys = default_b[0], default_b[1], "GCJ-02"

    with st.sidebar:
        st.subheader("🧱 障碍物")
        st.info("👉 直接在地图上点击画障碍物")
        if st.button("清空所有障碍物"):
            st.session_state.obstacles = []

        st.markdown("---")
        st.subheader("坐标系")
        coord = st.radio("", ["GCJ-02", "WGS-84"])
        if st.button("应用坐标系"):
            st.session_state.a_point = (a_lat_raw, a_lng_raw, coord)
            st.session_state.b_point = (b_lat_raw, b_lng_raw, coord)

    # 坐标转换
    if a_sys == "GCJ-02":
        a_lat, a_lng = gcj02_to_wgs84(a_lat_raw, a_lng_raw)
    else:
        a_lat, a_lng = a_lat_raw, a_lng_raw
    if b_sys == "GCJ-02":
        b_lat, b_lng = gcj02_to_wgs84(b_lat_raw, b_lng_raw)
    else:
        b_lat, b_lng = b_lat_raw, b_lng_raw

    # 绘制障碍物
    obs_js = ""
    for idx, o in enumerate(st.session_state.obstacles):
        latlngs = ", ".join([f"[{p[0]},{p[1]}]" for p in o])
        obs_js += f"L.polygon([{latlngs}],{{color:'orange',fillColor:'#ff7800',fillOpacity:0.5}}).addTo(map);"

    # 地图（完全无报错版）
    map_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
    <meta charset="utf-8">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>#map {{height:500px;width:100%;}}</style>
    </head>
    <body>
    <div id="map"></div>
    <script>
    var map = L.map('map').setView([{(a_lat+b_lat)/2},{(a_lng+b_lng)/2}],18);
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',{{attribution:'卫星'}}).addTo(map);

    L.marker([{a_lat},{a_lng}]).addTo(map).bindPopup("A起点");
    L.marker([{b_lat},{b_lng}]).addTo(map).bindPopup("B终点");
    L.polyline([[{a_lat},{a_lng}],[{b_lat},{b_lng}]],{{color:'blue',weight:5}}).addTo(map);

    {obs_js}

    // 手绘障碍物
    var points = [];
    map.on('click',(e)=>{{points.push([e.latlng.lat,e.latlng.lng]);}});
    map.on('dblclick',()=>{{
        if(points.length>2){{
            L.polygon(points,{{color:'orange',fillColor:'#ff7800',fillOpacity:0.5}}).addTo(map);
        }}
        points=[];
    }});
    </script>
    </body>
    </html>
    """

    st.subheader("卫星地图")
    html(map_html, width=700, height=500)

    # AB点设置
    c1,c2=st.columns(2)
    with c1:
        st.subheader("A点")
        la=st.number_input("纬度A",value=default_a[0],format="%.6f")
        ln=st.number_input("经度A",value=default_a[1],format="%.6f")
        if st.button("设置A"):
            st.session_state.a_point=(la,ln,coord)
    with c2:
        st.subheader("B点")
        lb=st.number_input("纬度B",value=default_b[0],format="%.6f")
        ll=st.number_input("经度B",value=default_b[1],format="%.6f")
        if st.button("设置B"):
            st.session_state.b_point=(lb,ll,coord)

    st.slider("飞行高度",10,200,50)

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    c1,c2=st.columns(2)
    with c1:
        if st.button("开始心跳"):
            st.session_state.simulation_on=True
    with c2:
        if st.button("停止心跳"):
            st.session_state.simulation_on=False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("✅ 在线")
    else:
        st.error("🚨 掉线")

    if not st.session_state.heartbeat_data.empty:
        st.line_chart(st.session_state.heartbeat_data.tail(50).set_index('时间')['序号'])