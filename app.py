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

# ================== 初始化所有状态 ==================
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

# ================== 心跳功能 ==================
def add_heartbeat():
    st.session_state.heartbeat_sequence += 1
    new_row = pd.DataFrame([{'序号': st.session_state.heartbeat_sequence, '时间': datetime.now()}])
    st.session_state.heartbeat_data = pd.concat([st.session_state.heartbeat_data, new_row], ignore_index=True)
    st.session_state.last_heartbeat_time = datetime.now()
    st.session_state.is_connected = True

def check_connection():
    if (datetime.now() - st.session_state.last_heartbeat_time).total_seconds() > 3:
        st.session_state.is_connected = False

# ================== 页面配置 ==================
st.set_page_config(page_title="无人机监控系统", layout="wide")
st.sidebar.title("📡 无人机导航")
page = st.sidebar.radio("选择页面", ["🗺️ 航线规划", "📶 飞行监控"])

# ================== 航线规划页面 ==================
if page == "🗺️ 航线规划":
    st.title("🗺️ 航线规划 & 障碍物设置")

    a_lat_raw, a_lng_raw, a_sys = st.session_state.a_point
    b_lat_raw, b_lng_raw, b_sys = st.session_state.b_point

    # 左侧控制面板
    with st.sidebar:
        st.markdown("---")
        st.subheader("🧱 障碍物操作")
        st.info("1. 点击地图点位\n2. 点击确认添加")
        
        if st.button("✅ 确认添加障碍物"):
            if st.session_state.click_lat and st.session_state.click_lng:
                height = random.randint(20, 60)
                st.session_state.obstacles.append((st.session_state.click_lat, st.session_state.click_lng, height))
                st.success(f"添加成功！高度：{height}m")
                st.session_state.click_lat = None
                st.session_state.click_lng = None
            else:
                st.warning("请先在地图上点击！")
        
        if st.button("🔄 清空所有障碍物"):
            st.session_state.obstacles = []
            st.success("已清空所有障碍物")

        st.markdown("---")
        st.subheader("🌐 坐标系")
        coord_sys = st.radio("坐标系", ["GCJ-02", "WGS-84"])
        if st.button("✅ 应用到 A、B 点"):
            st.session_state.a_point = (a_lat_raw, a_lng_raw, coord_sys)
            st.session_state.b_point = (b_lat_raw, b_lng_raw, coord_sys)
            st.success("应用成功！")

    # 坐标转换
    def wgs(lat, lng, sys):
        return gcj02_to_wgs84(lat, lng) if sys == "GCJ-02" else (lat, lng)
    a_lat, a_lng = wgs(a_lat_raw, a_lng_raw, a_sys)
    b_lat, b_lng = wgs(b_lat_raw, b_lng_raw, b_sys)

    # 障碍物绘制
    obs_js = ""
    for i, (lat, lng, h) in enumerate(st.session_state.obstacles):
        obs_js += f"""
        L.circle([{lat},{lng}],{{color:'orange',fillColor:'#ff8c00',fillOpacity:0.7,radius:15}}).addTo(map);
        L.marker([{lat},{lng}],{{icon:L.divIcon({{html:'{i+1}',className:'obs',iconSize:[22,22]}})}}).addTo(map);
        """

    temp_js = ""
    if st.session_state.click_lat:
        temp_js = f"L.marker([{st.session_state.click_lat},{st.session_state.click_lng}]).addTo(map);"

    # ================== 地图HTML（修复版，无报错）==================
    map_html = """
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>
            #map {width:100%;height:600px;}
            .obs {background:orange;color:white;font-weight:bold;border-radius:50%;text-align:center;line-height:22px;}
        </style>
    </head>
    <body>
        <div id="map"></div>
        <script>
            var map = L.map('map').setView(["""+str((a_lat+b_lat)/2)+""","""+str((a_lng+b_lng)/2)+"""], 18);
            L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: 'OpenStreetMap'
            }).addTo(map);

            L.marker(["""+str(a_lat)+","""+str(a_lng)+"""]).addTo(map).bindPopup('A起点');
            L.marker(["""+str(b_lat)+","""+str(b_lng)+"""]).addTo(map).bindPopup('B终点');
            L.polyline([["""+str(a_lat)+","""+str(a_lng)+"""],["""+str(b_lat)+","""+str(b_lng)+"""]],{color:'blue',weight:5}).addTo(map);
            
            """+obs_js+"""
            """+temp_js+"""
            
            map.on('click', function(e) {
                window.parent.postMessage({
                    type:'mapClick', lat:e.latlng.lat, lng:e.latlng.lng
                }, '*');
            });
        </script>
    </body>
    </html>
    """

    # 显示地图
    html(map_html, height=620)

    # 接收地图点击
    try:
        msg = st.components.v1.get_component_message("mapClick")
        if msg:
            st.session_state.click_lat = msg["lat"]
            st.session_state.click_lng = msg["lng"]
    except:
        pass

    # A、B点设置
    st.markdown("---")
    c1, c2 = st.columns(2)
    with c1:
        st.subheader("📍 A 点")
        a1 = st.number_input("A 纬度", a_lat_raw, format="%.6f")
        a2 = st.number_input("A 经度", a_lng_raw, format="%.6f")
        if st.button("✅ 设置 A 点"):
            st.session_state.a_point = (a1, a2, coord_sys)
    with c2:
        st.subheader("📍 B 点")
        b1 = st.number_input("B 纬度", b_lat_raw, format="%.6f")
        b2 = st.number_input("B 经度", b_lng_raw, format="%.6f")
        if st.button("✅ 设置 B 点"):
            st.session_state.b_point = (b1, b2, coord_sys)

    # 显示障碍物
    st.markdown("---")
    st.subheader(f"当前障碍物：{len(st.session_state.obstacles)} 个")
    if st.session_state.obstacles:
        with st.expander("查看障碍物详情"):
            for i, (lat, lng, h) in enumerate(st.session_state.obstacles):
                st.write(f"{i+1}. 坐标：{lat:.6f}, {lng:.6f}  高度：{h}m")

# ================== 飞行监控页面 ==================
else:
    st.title("📶 无人机心跳监控")

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳模拟"):
            st.session_state.simulation_on = True
    with c2:
        if st.button("⏸️ 停止模拟"):
            st.session_state.simulation_on = False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("✅ 无人机在线，心跳正常")
    else:
        st.error("🚨 无人机失联！")

    if not st.session_state.heartbeat_data.empty:
        last = st.session_state.heartbeat_data.iloc[-1]
        st.info(f"最新心跳：#{last['序号']}  {last['时间'].strftime('%H:%M:%S')}")

    st.subheader("📈 心跳趋势")
    df = st.session_state.heartbeat_data.tail(50)
    if not df.empty:
        st.line_chart(df.set_index("时间")["序号"])

    if st.button("🗑️ 清空心跳数据"):
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
        st.session_state.heartbeat_sequence = 0