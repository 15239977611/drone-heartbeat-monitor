import streamlit as st
import pandas as pd
import time
import math
import random
import json
import os
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

# ================== 记忆功能：本地文件存储障碍物 ==================
OBSTACLE_FILE = "obstacles.json"
def save_obstacles_to_file(obstacles):
    with open(OBSTACLE_FILE, "w", encoding="utf-8") as f:
        json.dump(obstacles, f, ensure_ascii=False, indent=2)

def load_obstacles_from_file():
    if os.path.exists(OBSTACLE_FILE):
        with open(OBSTACLE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    return []

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

# 障碍物状态（加载本地文件，实现记忆）
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = load_obstacles_from_file()
if 'obstacle_count' not in st.session_state:
    st.session_state.obstacle_count = 5

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

# ================== 航线规划页面 ==================
if page == "航线规划":
    st.title("🗺️ 航线规划 & 障碍物设置")
    st.markdown("设置起飞点A、降落点B，支持多边形圈选障碍物，数据自动保存在本地。")

    # 默认坐标（南京科技职业学院附近，GCJ-02）
    default_a = (32.2322, 118.7490)
    default_b = (32.2343, 118.7490)

    # 获取当前 A/B 点
    if 'a_point' in st.session_state:
        a_lat_raw, a_lng_raw, a_sys = st.session_state.a_point
    else:
        a_lat_raw, a_lng_raw, a_sys = default_a[0], default_a[1], "GCJ-02 (高德/百度)"
    if 'b_point' in st.session_state:
        b_lat_raw, b_lng_raw, b_sys = st.session_state.b_point
    else:
        b_lat_raw, b_lng_raw, b_sys = default_b[0], default_b[1], "GCJ-02 (高德/百度)"

    # 侧边栏：障碍物设置 + 坐标系
    with st.sidebar:
        st.subheader("⚙️ 障碍物设置")
        st.info("1. 点击地图获取坐标\n2. 输入顶点，点击添加多边形")

        # 障碍物输入
        with st.expander("添加多边形障碍物"):
            poly_str = st.text_area("输入多边形顶点（JSON格式）", 
                                    "[[32.233,118.749],[32.2331,118.7491],[32.2332,118.7490]]",
                                    help="输入顶点坐标列表，最后一个点会自动闭合")
            height = st.number_input("障碍物高度(m)", 20, 200, 50)
            if st.button("✅ 添加多边形障碍物"):
                try:
                    poly_points = json.loads(poly_str)
                    st.session_state.obstacles.append({
                        "points": poly_points,
                        "height": height
                    })
                    save_obstacles_to_file(st.session_state.obstacles)
                    st.success("添加成功！")
                except:
                    st.error("格式错误，请检查JSON格式")

        if st.button("🔄 清空所有障碍物"):
            st.session_state.obstacles = []
            save_obstacles_to_file([])
            st.success("已清空所有障碍物")

        st.markdown("---")
        st.subheader("🌐 坐标系设置")
        unified_coord_sys = st.radio("全局坐标系", ["GCJ-02 (高德/百度)", "WGS-84"], index=0)
        if st.button("✅ 确认并应用坐标系"):
            st.session_state.a_point = (a_lat_raw, a_lng_raw, unified_coord_sys)
            st.session_state.b_point = (b_lat_raw, b_lng_raw, unified_coord_sys)
            st.success("坐标系已同步到A、B点！")

    # 转换为 WGS-84
    if a_sys == "GCJ-02 (高德/百度)":
        a_lat_wgs, a_lng_wgs = gcj02_to_wgs84(a_lat_raw, a_lng_raw)
    else:
        a_lat_wgs, a_lng_wgs = a_lat_raw, a_lng_raw
    if b_sys == "GCJ-02 (高德/百度)":
        b_lat_wgs, b_lng_wgs = gcj02_to_wgs84(b_lat_raw, b_lng_raw)
    else:
        b_lat_wgs, b_lng_wgs = b_lat_raw, b_lng_raw

    obstacles = st.session_state.obstacles

    # ========== 构造卫星地图（显示多边形障碍物） ==========
    polygons_js = ""
    for i, obs in enumerate(obstacles):
        points = obs["points"]
        # 转换为Leaflet格式
        latlngs = ", ".join([f"[{p[0]}, {p[1]}]" for p in points])
        polygons_js += f"""
            L.polygon([{latlngs}], {{
                color: 'orange',
                fillColor: '#ff7800',
                fillOpacity: 0.5,
                weight: 2
            }}).addTo(map).bindPopup('障碍物 {i+1}<br>高度: {obs["height"]} 米');
        """

    map_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <title>卫星地图航线规划</title>
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>
            html, body, #map {{ margin: 0; height: 100%; width: 100%; }}
        </style>
    </head>
    <body>
        <div id="map" style="height: 500px; width: 100%;"></div>
        <script>
            var map = L.map('map', {{
                center: [{(a_lat_wgs + b_lat_wgs)/2}, {(a_lng_wgs + b_lng_wgs)/2}],
                zoom: 18,
                zoomControl: true,
                attributionControl: true
            }});

            // 高清卫星地图
            L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
                attribution: 'Leaflet | 卫星地图'
            }}).addTo(map);

            // A 点
            L.marker([{a_lat_wgs}, {a_lng_wgs}], {{
                icon: L.divIcon({{
                    html: '<div style="background-color: green; color: white; padding: 2px 6px; border-radius: 12px;">A 起点</div>',
                    className: 'custom-div-icon',
                    iconSize: [60, 25],
                    popupAnchor: [0, -10]
                }})
            }}).addTo(map).bindPopup('起点 A (校园内)');

            // B 点
            L.marker([{b_lat_wgs}, {b_lng_wgs}], {{
                icon: L.divIcon({{
                    html: '<div style="background-color: red; color: white; padding: 2px 6px; border-radius: 12px;">B 终点</div>',
                    className: 'custom-div-icon',
                    iconSize: [60, 25],
                    popupAnchor: [0, -10]
                }})
            }}).addTo(map).bindPopup('终点 B (校外)');

            // 航线
            var polyline = L.polyline([[{a_lat_wgs}, {a_lng_wgs}], [{b_lat_wgs}, {b_lng_wgs}]], {{
                color: 'blue',
                weight: 5,
                opacity: 0.8
            }}).addTo(map);
            polyline.bindPopup('航线');

            // 多边形障碍物
            {polygons_js}
        </script>
    </body>
    </html>
    """

    st.subheader("🗺️ 卫星地图（高清影像）")
    html(map_html, width=700, height=500)

    # 下方控件
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("起点 A（校园内）")
        a_lat_input = st.number_input("纬度 A", value=default_a[0], format="%.6f", key="a_lat")
        a_lng_input = st.number_input("经度 A", value=default_a[1], format="%.6f", key="a_lng")
        if st.button("设置 A 点", key="set_a"):
            st.session_state.a_point = (a_lat_input, a_lng_input, unified_coord_sys)
            st.success("✅ A点已保存")
    with col2:
        st.subheader("终点 B（校外）")
        b_lat_input = st.number_input("纬度 B", value=default_b[0], format="%.6f", key="b_lat")
        b_lng_input = st.number_input("经度 B", value=default_b[1], format="%.6f", key="b_lng")
        if st.button("设置 B 点", key="set_b"):
            st.session_state.b_point = (b_lat_input, b_lng_input, unified_coord_sys)
            st.success("✅ B点已保存")

    flight_height = st.slider("飞行高度 (米)", 10, 200, 50)

    st.markdown("---")
    st.subheader("系统状态")
    col3, col4 = st.columns(2)
    col3.metric("A点已设", "✅" if 'a_point' in st.session_state else "❌")
    col4.metric("B点已设", "✅" if 'b_point' in st.session_state else "❌")
    col5, _ = st.columns(2)
    col5.metric("障碍物数量", len(obstacles))

    with st.expander("📋 障碍物详细信息"):
        for i, obs in enumerate(obstacles):
            st.write(f"**障碍物 {i+1}**: 顶点数 {len(obs['points'])}, 高度 {obs['height']} 米")

# ================== 飞行监控页面 ==================
else:
    st.title("📡 飞行监控")
    st.markdown("实时心跳包监测（每秒一次），3秒未收到则报警。")

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
    else:
        st.info("暂无心跳数据，请点击「开始模拟心跳」")

    st.subheader("📈 心跳序号变化趋势（最近50次）")
    plot_data = st.session_state.heartbeat_data.tail(50).copy()
    if not plot_data.empty:
        plot_data['时间'] = pd.to_datetime(plot_data['时间'])
        st.line_chart(plot_data.set_index('时间')['序号'])
    else:
        st.info("暂无心跳数据，请点击「开始模拟心跳」")

    if st.button("🗑️ 清空历史心跳数据"):
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
        st.session_state.heartbeat_sequence = 0
        st.rerun()