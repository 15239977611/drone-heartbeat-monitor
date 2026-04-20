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

# 障碍物 & AB点
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []
if 'point_a' not in st.session_state:
    st.session_state.point_a = [32.2330, 118.7490]
if 'point_b' not in st.session_state:
    st.session_state.point_b = [32.2325, 118.7495]

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

# 侧边栏
with st.sidebar:
    st.header("导航")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("坐标系")
    coord = st.radio("", ["GCJ-02(高德/百度)", "WGS-84"])

    st.markdown("---")
    st.subheader("A点（起点）")
    a_lat = st.number_input("A纬度", value=st.session_state.point_a[0], format="%.6f")
    a_lng = st.number_input("A经度", value=st.session_state.point_a[1], format="%.6f")
    if st.button("更新A点位置"):
        st.session_state.point_a = [a_lat, a_lng]
    if st.button("清除A点"):
        st.session_state.point_a = []

    st.markdown("---")
    st.subheader("B点（终点）")
    b_lat = st.number_input("B纬度", value=st.session_state.point_b[0], format="%.6f")
    b_lng = st.number_input("B经度", value=st.session_state.point_b[1], format="%.6f")
    if st.button("更新B点位置"):
        st.session_state.point_b = [b_lat, b_lng]
    if st.button("清除B点"):
        st.session_state.point_b = []

    st.markdown("---")
    st.subheader("系统状态")
    if st.session_state.point_a:
        st.success("✅ A点已设")
    else:
        st.warning("❌ A点未设")
    if st.session_state.point_b:
        st.success("✅ B点已设")
    else:
        st.warning("❌ B点未设")

# ================== 航线规划 ==================
if page == "航线规划":
    st.title("🗺️ 卫星地图航线规划")

    # 坐标转换
    a_ok = len(st.session_state.point_a) == 2
    b_ok = len(st.session_state.point_b) == 2

    a_lat_t, a_lng_t = st.session_state.point_a if a_ok else (0,0)
    b_lat_t, b_lng_t = st.session_state.point_b if b_ok else (0,0)

    if coord == "GCJ-02(高德/百度)" and a_ok:
        a_lat, a_lng = gcj02_to_wgs84(a_lat_t, a_lng_t)
    else:
        a_lat, a_lng = a_lat_t, a_lng_t
    if coord == "GCJ-02(高德/百度)" and b_ok:
        b_lat, b_lng = gcj02_to_wgs84(b_lat_t, b_lng_t)
    else:
        b_lat, b_lng = b_lat_t, b_lng_t

    # 障碍物绘制
    obs_js = ""
    for o in st.session_state.obstacles:
        ps = ", ".join([f"[{p[0]},{p[1]}]" for p in o])
        obs_js += f"L.polygon([{ps}],{{color:'orange',fillColor:'orange',fillOpacity:0.5}}).addTo(map);"

    # 地图（支持拖拽AB点 + 显示图标）
    map_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
    <meta charset="utf-8">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>#map {{height:600px;width:100%;}}</style>
    </head>
    <body>
    <div id="map"></div>
    <script>
    var map = L.map('map').setView([{(a_lat+b_lat)/2},{(a_lng+b_lng)/2}],18);
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}').addTo(map);

    // A点（起点 绿色）
    var markerA = null;
    if({a_ok}){{
        markerA = L.marker([{a_lat},{a_lng}],{{
            draggable:true,
            icon:L.divIcon({{
                html:'<div style="background:green;color:white;padding:4px 8px;border-radius:8px;font-weight:bold;">A 起点</div>',
                iconSize:[100,30]
            }})
        }}).addTo(map);
    }}

    // B点（终点 红色）
    var markerB = null;
    if({b_ok}){{
        markerB = L.marker([{b_lat},{b_lng}],{{
            draggable:true,
            icon:L.divIcon({{
                html:'<div style="background:red;color:white;padding:4px 8px;border-radius:8px;font-weight:bold;">B 终点</div>',
                iconSize:[100,30]
            }})
        }}).addTo(map);
    }}

    // 航线
    if({a_ok} && {b_ok}){{
        L.polyline([[{a_lat},{a_lng}],[{b_lat},{b_lng}]],{{color:'blue',weight:4}}).addTo(map);
    }}

    // 障碍物
    {obs_js}

    // 手绘障碍物
    var points=[];
    map.on('click',e=>points.push([e.latlng.lat,e.latlng.lng]));
    map.on('dblclick',()=>{{
        if(points.length>2) L.polygon(points,{{color:'orange',fillColor:'orange',fillOpacity:0.5}}).addTo(map);
        points=[];
    }});
    </script>
    </body>
    </html>
    """

    html(map_html, height=600)

    st.subheader("操作说明")
    st.info("👉 可直接拖动地图上的A、B点移动位置；左侧可设置坐标、清除点位")
    st.success("A = 起点（绿色） | B = 终点（红色） | 蓝色线 = 自动航线")

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    c1,c2 = st.columns(2)
    with c1:
        if st.button("▶️ 开始心跳"):
            st.session_state.simulation_on=True
    with c2:
        if st.button("⏹️ 停止心跳"):
            st.session_state.simulation_on=False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("✅ 在线：心跳正常")
    else:
        st.error("🚨 警告：断开连接")

    if not st.session_state.heartbeat_data.empty:
        st.line_chart(st.session_state.heartbeat_data.tail(50).set_index('时间')['序号'])