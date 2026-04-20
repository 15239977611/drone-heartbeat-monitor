import streamlit as st
import pandas as pd
import time
import math
import random
from datetime import datetime
from streamlit.components.v1 import html

# ================== 坐标系转换 ==================
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
    return lat - dlat, lng - dlng

# ================== 初始化 ==================
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []
if 'click_lat' not in st.session_state:
    st.session_state.click_lat = None
if 'click_lng' not in st.session_state:
    st.session_state.click_lng = None

if 'a_point' not in st.session_state:
    st.session_state.a_point = (32.2322, 118.7490, "GCJ-02")
if 'b_point' not in st.session_state:
    st.session_state.b_point = (32.2343, 118.7490, "GCJ-02")

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

# ================== 心跳 ==================
def add_heartbeat():
    st.session_state.heartbeat_sequence += 1
    new_row = pd.DataFrame([{'序号': st.session_state.heartbeat_sequence, '时间': datetime.now()}])
    st.session_state.heartbeat_data = pd.concat([st.session_state.heartbeat_data, new_row], ignore_index=True)
    st.session_state.last_heartbeat_time = datetime.now()
    st.session_state.is_connected = True

def check_connection():
    if (datetime.now() - st.session_state.last_heartbeat_time).total_seconds() > 3:
        st.session_state.is_connected = False

# ================== 页面 ==================
st.set_page_config(page_title="无人机监控", layout="wide")
st.sidebar.title("导航")
page = st.sidebar.radio("页面", ["🗺️ 航线规划", "📶 飞行监控"])

# ================== 航线规划 ==================
if page == "🗺️ 航线规划":
    st.title("🗺️ 航线规划 & 障碍物")

    a_lat_raw, a_lng_raw, a_sys = st.session_state.a_point
    b_lat_raw, b_lng_raw, b_sys = st.session_state.b_point

    with st.sidebar:
        st.subheader("障碍物")
        if st.button("✅ 确认添加障碍物"):
            if st.session_state.click_lat and st.session_state.click_lng:
                st.session_state.obstacles.append((st.session_state.click_lat, st.session_state.click_lng, random.randint(20,60)))
                st.success("添加成功")
                st.session_state.click_lat=None
                st.session_state.click_lng=None
        if st.button("🔄 清空障碍物"):
            st.session_state.obstacles=[]

        st.subheader("坐标系")
        coord_sys = st.radio("坐标系", ["GCJ-02","WGS-84"])
        if st.button("应用到A/B点"):
            st.session_state.a_point = (a_lat_raw,a_lng_raw,coord_sys)
            st.session_state.b_point = (b_lat_raw,b_lng_raw,coord_sys)

    def wgsPoint(lat,lng,s):
        return gcj02_to_wgs84(lat,lng) if s=="GCJ-02" else (lat,lng)
    a_lat,a_lng = wgsPoint(a_lat_raw,a_lng_raw,a_sys)
    b_lat,b_lng = wgsPoint(b_lat_raw,b_lng_raw,b_sys)

    obs_js = ""
    for i,(lat,lng,h) in enumerate(st.session_state.obstacles):
        obs_js += f"L.circle([{lat},{lng}],{{color:'orange',fillOpacity:0.7,radius:15}}).addTo(map);"

    temp_js = ""
    if st.session_state.click_lat:
        temp_js = f"L.marker([{st.session_state.click_lat},{st.session_state.click_lng}]).addTo(map);"

    map_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>#map {{width:100%;height:600px;}}</style>
    </head>
    <body>
    <div id="map"></div>
    <script>
    var map = L.map('map').setView([{(a_lat+b_lat)/2},{(a_lng+b_lng)/2}],17);
    L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png').addTo(map);
    L.marker([{a_lat},{a_lng}]).addTo(map).bindPopup('A');
    L.marker([{b_lat},{b_lng}]).addTo(map).bindPopup('B');
    L.polyline([[{a_lat},{a_lng}],[{b_lat},{b_lng}]],{{color:'blue'}}).addTo(map);
    {obs_js}
    {temp_js}
    </script>
    </body>
    </html>
    """
    html(map_html, height=600)

    col1,col2=st.columns(2)
    with col1:
        st.number_input("A纬度",value=a_lat_raw,format="%.6f")
        st.number_input("A经度",value=a_lng_raw,format="%.6f")
    with col2:
        st.number_input("B纬度",value=b_lat_raw,format="%.6f")
        st.number_input("B经度",value=b_lng_raw,format="%.6f")

    st.subheader(f"障碍物：{len(st.session_state.obstacles)}")

# ================== 飞行监控 ==================
else:
    st.title("📶 心跳监控")
    c1,c2=st.columns(2)
    with c1:
        if st.button("开始"):
            st.session_state.simulation_on=True
    with c2:
        if st.button("停止"):
            st.session_state.simulation_on=False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("在线")
    else:
        st.error("失联")

    if not st.session_state.heartbeat_data.empty:
        st.line_chart(st.session_state.heartbeat_data.tail(50),x="时间",y="序号")