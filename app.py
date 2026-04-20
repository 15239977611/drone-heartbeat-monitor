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
    dlng = (dlng * 180.0) / (a / sqrtmagic * magic * math.pi)
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

# 障碍物存储
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []
if 'click_lat' not in st.session_state:
    st.session_state.click_lat = None
if 'click_lng' not in st.session_state:
    st.session_state.click_lng = None

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
    st.title("🗺️ 航线规划（点击地图添加障碍物）")

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

    # 侧边栏：障碍物 + 坐标系
    with st.sidebar:
        st.subheader("🧱 障碍物操作")
        st.info("👉 在地图上点击，再点确认添加")

        # 确认添加障碍物
        if st.button("✅ 确认添加该障碍物"):
            if st.session_state.click_lat and st.session_state.click_lng:
                h = random.randint(20, 50)
                st.session_state.obstacles.append([
                    st.session_state.click_lat,
                    st.session_state.click_lng,
                    h
                ])
                st.success(f"已添加障碍物，高度 {h}m")
                st.session_state.click_lat = None
                st.session_state.click_lng = None
            else:
                st.warning("请先在地图上点击一个位置")

        # 重置
        if st.button("🔄 清空所有障碍物"):
            st.session_state.obstacles = []
            st.success("已清空所有障碍物")

        st.markdown("---")
        st.subheader("🌐 坐标系")
        coord_sys = st.radio("坐标系", ["GCJ-02", "WGS-84"])
        if st.button("✅ 应用到 A、B 点"):
            st.session_state.a_point = (a_lat_raw, a_lng_raw, coord_sys)
            st.session_state.b_point = (b_lat_raw, b_lng_raw, coord_sys)
            st.success("坐标系已应用")

    # 坐标转换
    if a_sys == "GCJ-02":
        a_lat_wgs, a_lng_wgs = gcj02_to_wgs84(a_lat_raw, a_lng_raw)
    else:
        a_lat_wgs, a_lng_wgs = a_lat_raw, a_lng_raw

    if b_sys == "GCJ-02":
        b_lat_wgs, b_lng_wgs = gcj02_to_wgs84(b_lat_raw, b_lng_raw)
    else:
        b_lat_wgs, b_lng_wgs = b_lat_raw, b_lng_raw

    # 点击点（临时）
    click_js = ""
    if st.session_state.click_lat and st.session_state.click_lng:
        click_js = f"""
        L.marker([{st.session_state.click_lat},{st.session_state.click_lng}],{{
            icon:L.divIcon({{html:'待确认',className:'click-point',iconSize:[60,20]}})
        }}).addTo(map)
        """

    # 障碍物
    obs_js = ""
    for i, (lat, lng, h) in enumerate(st.session_state.obstacles):
        obs_js += f"""
        L.circle([{lat},{lng}],{{color:'orange',fillOpacity:0.5,radius:15}}).addTo(map)
        L.marker([{lat},{lng}],{{
            icon:L.divIcon({{html:'{i+1}',className:'obs-label',iconSize:[20,20]}})
        }}).addTo(map).bindPopup('障碍物{i+1}<br>高度{h}m')
        """

    # 地图（支持点击获取经纬度）
    map_html = f"""
    <html>
    <head>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        .obs-label{{background:orange;color:white;font-weight:bold;border-radius:50%;text-align:center;line-height:20px;font-size:12px}}
        .click-point{{background:#222;color:white;padding:2px 6px;border-radius:8px;font-size:12px}}
    </style>
    </head>
    <body>
    <div id="map" style="width:100%;height:500px"></div>
    <script>
        var map = L.map('map').setCenter([{(a_lat_wgs+b_lat_wgs)/2},{(a_lng_wgs+b_lng_wgs)/2}],18);
        L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}').addTo(map);

        // AB点
        L.marker([{a_lat_wgs},{a_lng_wgs}],{{
            icon:L.divIcon({{html:'A 起点',className:'ab-point',iconSize:[60,24]}})
        }}).addTo(map)
        L.marker([{b_lat_wgs},{b_lng_wgs}],{{
            icon:L.divIcon({{html:'B 终点',className:'ab-point',iconSize:[60,24]}})
        }}).addTo(map)
        L.polyline([[{a_lat_wgs},{a_lng_wgs}],[{b_lat_wgs},{b_lng_wgs}]],{{color:'blue',weight:5}}).addTo(map);

        // 障碍物
        {obs_js}
        // 临时点击点
        {click_js}

        // 点击获取坐标
        map.on('click',function(e){{
            const lat = e.latlng.lat;
            const lng = e.latlng.lng;
            window.parent.postMessage({{type:'map_click',lat:lat,lng:lng}},'*');
        }});
    </script>
    </body>
    </html>
    """

    # 接收地图点击
    components_html = html(map_html, width=700, height=500)
    js_msg = st.components.v1.get_component_message("map_click")
    if js_msg:
        st.session_state.click_lat = js_msg["lat"]
        st.session_state.click_lng = js_msg["lng"]

    # AB点设置
    c1, c2 = st.columns(2)
    with c1:
        st.subheader("A点")
        a_lat = st.number_input("纬度A", value=a_lat_raw, format="%.6f")
        a_lng = st.number_input("经度A", value=a_lng_raw, format="%.6f")
        if st.button("设置A点"):
            st.session_state.a_point = (a_lat, a_lng, coord_sys)
    with c2:
        st.subheader("B点")
        b_lat = st.number_input("纬度B", value=b_lat_raw, format="%.6f")
        b_lng = st.number_input("经度B", value=b_lng_raw, format="%.6f")
        if st.button("设置B点"):
            st.session_state.b_point = (b_lat, b_lng, coord_sys)

    st.slider("飞行高度", 10, 200, 50)

    # 显示障碍物
    st.markdown("---")
    st.subheader(f"当前障碍物总数：{len(st.session_state.obstacles)}")
    if st.session_state.obstacles:
        with st.expander("查看详情"):
            for i, (lat, lng, h) in enumerate(st.session_state.obstacles):
                st.write(f"{i+1}.  lat={lat:.6f} lng={lng:.6f} 高度={h}m")

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控 & 心跳监测")

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始模拟心跳"):
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
        st.success("✅ 心跳正常，无人机在线")
    else:
        st.error("🚨 超时未收到心跳！无人机失联")

    if not st.session_state.heartbeat_data.empty:
        last = st.session_state.heartbeat_data.iloc[-1]
        st.info(f"最新心跳：#{last['序号']}  {last['时间'].strftime('%H:%M:%S')}")

    st.subheader("心跳趋势")
    d = st.session_state.heartbeat_data.tail(50)
    if not d.empty:
        st.line_chart(d.set_index("时间")["序号"])

    if st.button("清空心跳数据"):
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
        st.session_state.heartbeat_sequence = 0