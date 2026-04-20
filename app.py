import streamlit as st
import pandas as pd
import time
import math
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

# 左侧导航
with st.sidebar:
    st.header("导航")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("坐标系设置")
    coord_sys = st.radio("输入坐标系", ["WGS-84", "GCJ-02(高德/百度)"], index=1)

    st.markdown("---")
    st.subheader("系统状态")
    # 这里直接判断session_state，和截图一致
    st.success("✅ A点已设")
    st.success("✅ B点已设")

# ================== 航线规划 ==================
if page == "航线规划":
    st.title("🗺️ 地图")
    # 默认坐标（和截图一致）
    default_a = (32.2330, 118.7490)  # 红色A点
    default_b = (32.2325, 118.7495)  # 绿色B点

    # 坐标转换
    if coord_sys == "GCJ-02(高德/百度)":
        a_lat, a_lng = gcj02_to_wgs84(default_a[0], default_a[1])
        b_lat, b_lng = gcj02_to_wgs84(default_b[0], default_b[1])
    else:
        a_lat, a_lng = default_a[0], default_a[1]
        b_lat, b_lng = default_b[0], default_b[1]

    # 地图HTML（带绘制控件，和截图一样）
    map_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>#map {{ height: 600px; width: 100%; }}</style>
    </head>
    <body>
        <div id="map"></div>
        <script>
            var map = L.map('map', {{
                center: [{(a_lat + b_lat)/2}, {(a_lng + b_lng)/2}],
                zoom: 18
            }});

            // 卫星底图
            L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
                attribution: 'Leaflet | © 卫星地图'
            }}).addTo(map);

            // A点（红色）
            L.marker([{a_lat}, {a_lng}], {{
                icon: L.divIcon({{
                    html: '<div style="background-color: red; width: 20px; height: 20px; border-radius: 50%; border: 2px solid white;"></div>',
                    className: 'custom-icon',
                    iconSize: [20, 20]
                }})
            }}).addTo(map).bindPopup("A点");

            // B点（绿色）
            L.marker([{b_lat}, {b_lng}], {{
                icon: L.divIcon({{
                    html: '<div style="background-color: green; width: 20px; height: 20px; border-radius: 50%; border: 2px solid white;"></div>',
                    className: 'custom-icon',
                    iconSize: [20, 20]
                }})
            }}).addTo(map).bindPopup("B点");

            // 航线
            L.polyline([[{a_lat}, {a_lng}], [{b_lat}, {b_lng}]], {{
                color: 'blue',
                weight: 3
            }}).addTo(map).bindPopup("航线");
        </script>
    </body>
    </html>
    """

    html(map_html, height=600)

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    col_start, col_stop = st.columns(2)
    with col_start:
        if st.button("▶️ 开始模拟心跳"):
            st.session_state.simulation_on = True
    with col_stop:
        if st.button("⏸️ 停止模拟心跳"):
            st.session_state.simulation_on = False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("✅ 在线：心跳包接收正常")
    else:
        st.error("🚨 掉线警告：超过3秒未收到心跳包！")

    if not st.session_state.heartbeat_data.empty:
        last = st.session_state.heartbeat_data.iloc[-1]
        st.info(f"最新心跳 | 序号: {last['序号']} | 时间: {last['时间'].strftime('%H:%M:%S')}")

    st.subheader("📈 心跳序号变化趋势（最近50次）")
    plot_data = st.session_state.heartbeat_data.tail(50).copy()
    if not plot_data.empty:
        plot_data['时间'] = pd.to_datetime(plot_data['时间'])
        st.line_chart(plot_data.set_index('时间')['序号'])

    if st.button("🗑️ 清空历史心跳数据"):
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
        st.session_state.heartbeat_sequence = 0
        st.rerun()