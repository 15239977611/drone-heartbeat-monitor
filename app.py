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

# ================== 障碍物存储 ==================
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []
if 'click_lat' not in st.session_state:
    st.session_state.click_lat = None
if 'click_lng' not in st.session_state:
    st.session_state.click_lng = None

# ================== 心跳函数 ==================
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
st.sidebar.title("📡 无人机导航")
page = st.sidebar.radio("选择页面", ["🗺️ 航线规划", "📶 飞行监控"])

# ================== 航线规划页面 ==================
if page == "🗺️ 航线规划":
    st.title("🗺️ 无人机航线规划 & 障碍物设置")

    # 默认坐标
    default_a = (32.2322, 118.7490)
    default_b = (32.2343, 118.7490)

    # 读取A、B点
    if 'a_point' in st.session_state:
        a_lat_raw, a_lng_raw, a_sys = st.session_state.a_point
    else:
        a_lat_raw, a_lng_raw, a_sys = default_a[0], default_a[1], "GCJ-02"

    if 'b_point' in st.session_state:
        b_lat_raw, b_lng_raw, b_sys = st.session_state.b_point
    else:
        b_lat_raw, b_lng_raw, b_sys = default_b[0], default_b[1], "GCJ-02"

    # ================== 左侧控制面板 ==================
    with st.sidebar:
        st.markdown("---")
        st.subheader("🧱 障碍物操作")
        st.info("1. 点击地图点位\n2. 点击确认添加")

        if st.button("✅ 确认添加障碍物"):
            if st.session_state.click_lat and st.session_state.click_lng:
                height = random.randint(20, 50)
                st.session_state.obstacles.append([
                    st.session_state.click_lat,
                    st.session_state.click_lng,
                    height
                ])
                st.success(f"添加成功！高度：{height}m")
                st.session_state.click_lat = None
                st.session_state.click_lng = None
            else:
                st.warning("请先点击地图！")

        if st.button("🔄 清空所有障碍物"):
            st.session_state.obstacles = []
            st.success("已清空所有障碍物")

        st.markdown("---")
        st.subheader("🌐 坐标系设置")
        coord_sys = st.radio("选择坐标系", ["GCJ-02 (高德/百度)", "WGS-84"])
        if st.button("✅ 应用到 A、B 点"):
            st.session_state.a_point = (a_lat_raw, a_lng_raw, coord_sys)
            st.session_state.b_point = (b_lat_raw, b_lng_raw, coord_sys)
            st.success("坐标系已应用！")

    # 坐标转换
    if a_sys == "GCJ-02 (高德/百度)":
        a_lat_wgs, a_lng_wgs = gcj02_to_wgs84(a_lat_raw, a_lng_raw)
    else:
        a_lat_wgs, a_lng_wgs = a_lat_raw, a_lng_raw

    if b_sys == "GCJ-02 (高德/百度)":
        b_lat_wgs, b_lng_wgs = gcj02_to_wgs84(b_lat_raw, b_lng_raw)
    else:
        b_lat_wgs, b_lng_wgs = b_lat_raw, b_lng_raw

    # 障碍物JS
    obs_js = ""
    for i, (lat, lng, h) in enumerate(st.session_state.obstacles):
        obs_js += f"""
        L.circle([{lat},{lng}],{{
            color:'orange', fillColor:'#ff9900', fillOpacity:0.6, radius:18
        }}).addTo(map);
        L.marker([{lat},{lng}],{{
            icon:L.divIcon({{
                html:'{i+1}',
                className:'obs-label',
                iconSize:[24,24]
            }})
        }}).addTo(map).bindPopup('障碍物{i+1}<br>高度：{h}米');
        """

    # 临时点击点
    temp_js = ""
    if st.session_state.click_lat and st.session_state.click_lng:
        temp_js = f"""
        L.marker([{st.session_state.click_lat},{st.session_state.click_lng}],{{
            icon:L.divIcon({{html:'📍 待确认', className:'temp-point', iconSize:[70,22]}})
        }}).addTo(map);
        """

    # ================== 地图 ==================
    map_html = f"""
    <html>
    <head>
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>
            .obs-label {{
                background:orange; color:white; font-weight:bold;
                border-radius:50%; text-align:center; line-height:24px;
                font-size:13px; border:2px solid white;
            }}
            .temp-point {{
                background:#222; color:white; padding:2px 6px; border-radius:6px; font-size:12px;
            }}
        </style>
    </head>
    <body>
        <div id="map" style="width:100%; height:550px;"></div>
        <script>
            var map = L.map('map').setCenter([{(a_lat_wgs + b_lat_wgs)/2},{(a_lng_wgs + b_lng_wgs)/2}], 18);
            L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}').addTo(map);

            // AB点
            L.marker([{a_lat_wgs},{a_lng_wgs}],{{
                icon:L.divIcon({{html:'A 起点', className:'ab', iconSize:[65,25]}})
            }}).addTo(map);
            L.marker([{b_lat_wgs},{b_lng_wgs}],{{
                icon:L.divIcon({{html:'B 终点', className:'ab', iconSize:[65,25]}})
            }}).addTo(map);
            L.polyline([[{a_lat_wgs},{a_lng_wgs}],[{b_lat_wgs},{b_lng_wgs}]],{{
                color:'#0066ff', weight:5, opacity:0.8
            }}).addTo(map);

            // 障碍物
            {obs_js}
            // 临时点击点
            {temp_js}

            // 点击获取坐标
            map.on('click', function(e) {{
                window.parent.postMessage({{
                    type: 'mapClick',
                    lat: e.latlng.lat,
                    lng: e.latlng.lng
                }}, '*');
            }});
        </script>
    </body>
    </html>
    """

    # 渲染地图
    map_component = html(map_html, width=1000, height=550)

    # 获取点击事件
    try:
        msg = st.components.v1.get_component_message("mapClick")
        if msg:
            st.session_state.click_lat = msg["lat"]
            st.session_state.click_lng = msg["lng"]
    except:
        pass

    # ================== AB点设置 ==================
    st.markdown("---")
    colA, colB = st.columns(2)
    with colA:
        st.subheader("📍 起点 A")
        a_lat = st.number_input("纬度 A", value=a_lat_raw, format="%.6f")
        a_lng = st.number_input("经度 A", value=a_lng_raw, format="%.6f")
        if st.button("✅ 设置 A 点"):
            st.session_state.a_point = (a_lat, a_lng, coord_sys)
            st.success("A 点已保存")

    with colB:
        st.subheader("📍 终点 B")
        b_lat = st.number_input("纬度 B", value=b_lat_raw, format="%.6f")
        b_lng = st.number_input("经度 B", value=b_lng_raw, format="%.6f")
        if st.button("✅ 设置 B 点"):
            st.session_state.b_point = (b_lat, b_lng, coord_sys)
            st.success("B 点已保存")

    # 障碍物列表
    st.markdown("---")
    st.subheader(f"📦 当前障碍物总数：{len(st.session_state.obstacles)}")
    if st.session_state.obstacles:
        with st.expander("查看障碍物详情"):
            for i, (lat, lng, h) in enumerate(st.session_state.obstacles):
                st.write(f"**{i+1}** → 纬度：`{lat:.6f}` 经度：`{lng:.6f}` 高度：`{h}m`")

# ================== 飞行监控页面 ==================
else:
    st.title("📶 无人机实时心跳监控")

    c1, c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳模拟"):
            st.session_state.simulation_on = True
    with c2:
        if st.button("⏸️ 停止心跳模拟"):
            st.session_state.simulation_on = False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    # 连接状态
    if st.session_state.is_connected:
        st.success("✅ 无人机在线 — 心跳接收正常")
    else:
        st.error("🚨 无人机失联 — 超过3秒未收到心跳！")

    # 最新心跳
    if not st.session_state.heartbeat_data.empty:
        last = st.session_state.heartbeat_data.iloc[-1]
        st.info(f"最新心跳：序号 {last['序号']} | 时间 {last['时间'].strftime('%H:%M:%S')}")

    # 图表
    st.subheader("📈 心跳序号趋势")
    df = st.session_state.heartbeat_data.tail(50)
    if not df.empty:
        df['时间'] = pd.to_datetime(df['时间'])
        st.line_chart(df.set_index('时间')['序号'])

    # 清空
    if st.button("🗑️ 清空所有心跳数据"):
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
        st.session_state.heartbeat_sequence = 0
        st.rerun()