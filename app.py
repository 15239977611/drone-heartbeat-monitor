import streamlit as st
import folium
from streamlit_folium import st_folium
import plotly.graph_objects as go
import pandas as pd
import numpy as np
import time
import json
import os
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import unary_union

# ===================== 1. 工具函数：GCJ-02 <-> WGS84 转换 =====================
def gcj02_to_wgs84(lat, lon):
    a = 6378245.0
    ee = 0.006693421622965943
    dlat = _transform_lat(lon - 105.0, lat - 35.0)
    dlon = _transform_lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * np.pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * np.pi)
    dlon = (dlon * 180.0) / (a / sqrtmagic * np.cos(radlat) * np.pi)
    mglat = lat + dlat
    mglon = lon + dlon
    return lat * 2 - mglat, lon * 2 - mglon

def wgs84_to_gcj02(lat, lon):
    a = 6378245.0
    ee = 0.006693421622965943
    dlat = _transform_lat(lon - 105.0, lat - 35.0)
    dlon = _transform_lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * np.pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * np.pi)
    dlon = (dlon * 180.0) / (a / sqrtmagic * np.cos(radlat) * np.pi)
    mglat = lat + dlat
    mglon = lon + dlon
    return mglat, mglon

def _transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * np.sqrt(abs(x))
    ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
    ret += (20.0 * np.sin(y * np.pi) + 40.0 * np.sin(y / 3.0 * np.pi)) * 2.0 / 3.0
    ret += (160.0 * np.sin(y / 12.0 * np.pi) + 320 * np.sin(y * np.pi / 30.0)) * 2.0 / 3.0
    return ret

def _transform_lon(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * np.sqrt(abs(x))
    ret += (20.0 * np.sin(6.0 * x * np.pi) + 20.0 * np.sin(2.0 * x * np.pi)) * 2.0 / 3.0
    ret += (20.0 * np.sin(x * np.pi) + 40.0 * np.sin(x / 3.0 * np.pi)) * 2.0 / 3.0
    ret += (150.0 * np.sin(x / 12.0 * np.pi) + 300.0 * np.sin(x / 30.0 * np.pi)) * 2.0 / 3.0
    return ret

# ===================== 2. 页面配置与状态初始化 =====================
st.set_page_config(page_title="无人机智能化应用Demo", layout="wide")
st.sidebar.title("导航")
page = st.sidebar.radio("功能页面", ["航线规划", "飞行监控"])

# 校园中心点（GCJ-02）
BASE_LAT_GCJ = 32.2322
BASE_LON_GCJ = 118.7494
BASE_LAT_WGS, BASE_LON_WGS = gcj02_to_wgs84(BASE_LAT_GCJ, BASE_LON_GCJ)

# 状态初始化
if "a_point_gcj" not in st.session_state:
    st.session_state.a_point_gcj = None
if "b_point_gcj" not in st.session_state:
    st.session_state.b_point_gcj = None
if "obstacles" not in st.session_state:
    # 障碍物格式：[{"poly": Polygon, "height": float, "gcj_coords": list}]
    st.session_state.obstacles = []
if "flight_altitude" not in st.session_state:
    st.session_state.flight_altitude = 50
if "route_points" not in st.session_state:
    st.session_state.route_points = []
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
if "last_heartbeat_time" not in st.session_state:
    st.session_state.last_heartbeat_time = time.time()
if "flight_status" not in st.session_state:
    st.session_state.flight_status = "未开始"
if "current_waypoint" not in st.session_state:
    st.session_state.current_waypoint = 0

# 障碍物数据持久化
OBSTACLES_FILE = "obstacles.json"
if os.path.exists(OBSTACLES_FILE):
    with open(OBSTACLES_FILE, "r") as f:
        obs_data = json.load(f)
    st.session_state.obstacles = []
    for obs in obs_data:
        poly = Polygon(obs["coords"])
        st.session_state.obstacles.append({
            "poly": poly,
            "height": obs["height"],
            "gcj_coords": obs["coords"]
        })

# ===================== 3. 工具函数：绕飞算法 =====================
def generate_avoid_route(a_lat, a_lon, b_lat, b_lon, obs_list, safe_distance=0.001):
    """生成带安全距离的绕飞航线，支持左右绕飞"""
    line = LineString([(a_lat, a_lon), (b_lat, b_lon)])
    obstacles = [obs["poly"].buffer(safe_distance) for obs in obs_list]
    union_obstacles = unary_union(obstacles)
    
    if not line.intersects(union_obstacles):
        return [
            {"lat": a_lat, "lon": a_lon, "alt": st.session_state.flight_altitude},
            {"lat": b_lat, "lon": b_lon, "alt": st.session_state.flight_altitude}
        ]
    
    # 简单左右绕飞实现
    mid_lat = (a_lat + b_lat) / 2
    mid_lon = (a_lon + b_lon) / 2
    offset = safe_distance * 2
    
    # 左右两个候选点
    left_point = (mid_lat + offset, mid_lon)
    right_point = (mid_lat - offset, mid_lon)
    
    # 选不穿障碍物的一侧
    if not LineString([(a_lat,a_lon), left_point, (b_lat,b_lon)]).intersects(union_obstacles):
        return [
            {"lat": a_lat, "lon": a_lon, "alt": st.session_state.flight_altitude},
            {"lat": left_point[0], "lon": left_point[1], "alt": st.session_state.flight_altitude},
            {"lat": b_lat, "lon": b_lon, "alt": st.session_state.flight_altitude}
        ]
    else:
        return [
            {"lat": a_lat, "lon": a_lon, "alt": st.session_state.flight_altitude},
            {"lat": right_point[0], "lon": right_point[1], "alt": st.session_state.flight_altitude},
            {"lat": b_lat, "lon": b_lon, "alt": st.session_state.flight_altitude}
        ]

# ===================== 4. 页面1：航线规划 =====================
if page == "航线规划":
    st.title("✈️ 航线规划 - 校园无人机作业")
    
    # 坐标系选择
    coord_system = st.radio("输入坐标系", ["GCJ-02(高德/百度)", "WGS-84"], index=0)
    
    # 控制面板
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("起点A")
        a_lat = st.number_input("纬度", value=BASE_LAT_GCJ if coord_system == "GCJ-02(高德/百度)" else BASE_LAT_WGS, format="%.6f")
        a_lon = st.number_input("经度", value=BASE_LON_GCJ if coord_system == "GCJ-02(高德/百度)" else BASE_LON_WGS, format="%.6f")
        if st.button("设置A点"):
            if coord_system == "GCJ-02(高德/百度)":
                st.session_state.a_point_gcj = (a_lat, a_lon)
            else:
                gcj_lat, gcj_lon = wgs84_to_gcj02(a_lat, a_lon)
                st.session_state.a_point_gcj = (gcj_lat, gcj_lon)
            st.success("✅ A点已设置")
    
    with col2:
        st.subheader("终点B")
        b_lat = st.number_input("纬度", value=BASE_LAT_GCJ + 0.001 if coord_system == "GCJ-02(高德/百度)" else BASE_LAT_WGS + 0.001, format="%.6f")
        b_lon = st.number_input("经度", value=BASE_LON_GCJ + 0.001 if coord_system == "GCJ-02(高德/百度)" else BASE_LON_WGS + 0.001, format="%.6f")
        if st.button("设置B点"):
            if coord_system == "GCJ-02(高德/百度)":
                st.session_state.b_point_gcj = (b_lat, b_lon)
            else:
                gcj_lat, gcj_lon = wgs84_to_gcj02(b_lat, b_lon)
                st.session_state.b_point_gcj = (gcj_lat, gcj_lon)
            st.success("✅ B点已设置")
    
    # 飞行参数
    st.subheader("飞行参数")
    st.session_state.flight_altitude = st.slider("设定飞行高度(m)", min_value=10, max_value=200, value=50)
    
    # 障碍物圈选
    st.subheader("障碍物圈选（多边形）")
    obstacle_height = st.number_input("障碍物高度(m)", value=30)
    if st.button("添加障碍物（地图点击圈选）"):
        st.info("请在地图上点击多个点，形成多边形障碍物，最后点击第一个点闭合")
    
    # 地图显示（卫星底图）
    st.subheader("🗺️ 校园卫星地图")
    tiles = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    m = folium.Map(location=[BASE_LAT_GCJ, BASE_LON_GCJ], zoom_start=16, tiles=tiles, attr="卫星影像")
    
    # 绘制已设置的A/B点
    if st.session_state.a_point_gcj:
        folium.Marker(st.session_state.a_point_gcj, popup="起点A", icon=folium.Icon(color="green")).add_to(m)
    if st.session_state.b_point_gcj:
        folium.Marker(st.session_state.b_point_gcj, popup="终点B", icon=folium.Icon(color="red")).add_to(m)
    
    # 绘制障碍物
    for obs in st.session_state.obstacles:
        folium.Polygon(locations=obs["gcj_coords"], color="red", fill=True, fill_color="red", fill_opacity=0.4, popup=f"障碍物 高度:{obs['height']}m").add_to(m)
    
    # 绘制航线
    if st.session_state.route_points:
        route_locs = [(p["lat"], p["lon"]) for p in st.session_state.route_points]
        folium.PolyLine(route_locs, color="blue", weight=4, opacity=0.8).add_to(m)
    
    map_data = st_folium(m, width=1000, height=600)
    
    # 生成绕飞航线
    if st.button("生成绕飞航线") and st.session_state.a_point_gcj and st.session_state.b_point_gcj:
        a_lat_gcj, a_lon_gcj = st.session_state.a_point_gcj
        b_lat_gcj, b_lon_gcj = st.session_state.b_point_gcj
        route = generate_avoid_route(a_lat_gcj, a_lon_gcj, b_lat_gcj, b_lon_gcj, st.session_state.obstacles)
        st.session_state.route_points = route
        st.success("✅ 绕飞航线已生成！")
    
    # 保存障碍物
    if st.button("保存障碍物配置"):
        obs_data = []
        for obs in st.session_state.obstacles:
            obs_data.append({
                "coords": obs["gcj_coords"],
                "height": obs["height"]
            })
        with open(OBSTACLES_FILE, "w") as f:
            json.dump(obs_data, f)
        st.success("✅ 障碍物配置已保存")

# ===================== 5. 页面2：飞行监控 =====================
elif page == "飞行监控":
    st.title("🛰️ 飞行监控 - 实时状态与心跳包")
    
    # 心跳包模拟与掉线检测
    st.subheader("💓 无人机心跳包监测")
    heartbeat_placeholder = st.empty()
    status_placeholder = st.empty()
    
    # 模拟心跳包
    current_time = time.time()
    if len(st.session_state.heartbeat_data) == 0 or current_time - st.session_state.last_heartbeat_time >= 1:
        seq = len(st.session_state.heartbeat_data) + 1
        st.session_state.heartbeat_data.append({
            "seq": seq,
            "time": time.strftime("%H:%M:%S"),
            "timestamp": current_time
        })
        st.session_state.last_heartbeat_time = current_time
    
    # 掉线检测
    if current_time - st.session_state.last_heartbeat_time > 3:
        status_placeholder.error("⚠️ 连接超时！3秒未收到心跳包")
    else:
        status_placeholder.success("✅ 连接正常，心跳包接收中")
    
    # 心跳包折线图
    df_hb = pd.DataFrame(st.session_state.heartbeat_data)
    if not df_hb.empty:
        heartbeat_placeholder.line_chart(df_hb.set_index("time")["seq"], use_container_width=True)
    
    # 飞行状态监控
    st.subheader("📊 飞行任务监控")
    col1, col2, col3, col4, col5 = st.columns(5)
    with col1:
        st.metric("当前航点", f"{st.session_state.current_waypoint}/{len(st.session_state.route_points)}")
    with col2:
        st.metric("飞行速度", "8.5 m/s")
    with col3:
        st.metric("已用时间", "00:43")
    with col4:
        st.metric("剩余距离", "0 m")
    with col5:
        st.metric("电量模拟", "96%")
    
    # 任务控制按钮
    col_start, col_pause, col_stop = st.columns(3)
    with col_start:
        if st.button("开始任务"):
            st.session_state.flight_status = "飞行中"
            st.session_state.current_waypoint = 1
    with col_pause:
        if st.button("暂停"):
            st.session_state.flight_status = "已暂停"
    with col_stop:
        if st.button("停止/重置"):
            st.session_state.flight_status = "未开始"
            st.session_state.current_waypoint = 0
    
    # 实时飞行地图
    st.subheader("🗺️ 实时飞行地图")
    m_flight = folium.Map(location=[BASE_LAT_GCJ, BASE_LON_GCJ], zoom_start=16, tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}")
    if st.session_state.route_points:
        route_locs = [(p["lat"], p["lon"]) for p in st.session_state.route_points]
        folium.PolyLine(route_locs, color="blue", weight=4).add_to(m_flight)
        # 绘制当前位置
        if st.session_state.current_waypoint > 0 and st.session_state.current_waypoint <= len(st.session_state.route_points):
            current_p = st.session_state.route_points[st.session_state.current_waypoint - 1]
            folium.Marker([current_p["lat"], current_p["lon"]], popup="当前位置", icon=folium.Icon(color="orange")).add_to(m_flight)
    st_folium(m_flight, width=1000, height=400)