import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point, LinearRing
from shapely.ops import nearest_points, unary_union
from shapely.affinity import scale
import math
from datetime import datetime, timedelta
import time
import random
import plotly.graph_objects as go
import pandas as pd

# ================== 坐标系转换核心算法（WGS84 ↔ GCJ02） ==================
PI = math.pi
a = 6378245.0  # 长半轴
ee = 0.00669342162296594323  # 偏心率平方

def transform_lat(x, y):
    """纬度转换"""
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * PI) + 320 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(x, y):
    """经度转换"""
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return ret

def out_of_china(lat, lng):
    """判断是否在中国境内（境外不转换）"""
    return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)

def wgs84_to_gcj02(lng, lat):
    """WGS84转GCJ02（火星坐标系）"""
    if out_of_china(lat, lng):
        return [lng, lat]
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * PI)
    mglat = lat + dlat
    mglng = lng + dlng
    return [mglng, mglat]

def gcj02_to_wgs84(lng, lat):
    """GCJ02转WGS84（高精度）"""
    if out_of_china(lat, lng):
        return [lng, lat]
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * PI)
    mglat = lat + dlat
    mglng = lng + dlng
    return [lng * 2 - mglng, lat * 2 - mglat]

# ================== 初始化（重点强化安全半径） ==================
# 基础初始化
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles_all" not in st.session_state:
    st.session_state.obstacles_all = []
if "obstacles_type" not in st.session_state:
    st.session_state.obstacles_type = []
if "obstacles_height" not in st.session_state:
    st.session_state.obstacles_height = []
if "drawing_mode" not in st.session_state:
    st.session_state.drawing_mode = None
if "current_points" not in st.session_state:
    st.session_state.current_points = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 8  # 默认8米

# ========== 重点：安全半径初始化（强制显示） ==========
if "drone_safety_radius" not in st.session_state:
    st.session_state.drone_safety_radius = 15  # 默认15米
# 安全半径单位转换（关键）
SAFE_DISTANCE_LNG_LAT_PER_METER = 1 / 111000  # 1米对应的经纬度值

# 心跳包状态
if "drone_heartbeat" not in st.session_state:
    st.session_state.drone_heartbeat = {
        "last_time": datetime.now(),
        "signal_strength": 95,
        "battery": 88,
        "gps_status": "正常",
        "flight_status": "待命",
        "latitude": 32.2330,
        "longitude": 118.7490,
        "speed": 0.0,
        "heartbeat_interval": 1,
        "heartbeat_seq": 0
    }
if "heartbeat_log" not in st.session_state:
    st.session_state.heartbeat_log = []
if "heartbeat_chart_data" not in st.session_state:
    st.session_state.heartbeat_chart_data = {"time": [], "seq": []}
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False
# 坐标系状态
if "coord_system" not in st.session_state:
    st.session_state.coord_system = "WGS84（原始GPS）"
if "transformed_points" not in st.session_state:
    st.session_state.transformed_points = {
        "point_a": None,
        "point_b": None,
        "obstacles": []
    }

# ================== 核心配置 ==================
GROUND_HEIGHT = 0  # 地面基准高度
REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50,
    "普通房屋": 20,
    "高层楼房": 80,
    "大树/电线杆": 10,
    "操场/空地": 0,
    "桥梁/高架": 15,
    "塔楼/信号塔": 60
}

# ================== 永久存储函数 ==================
def save_all():
    with open("geo_obstacles.json", "w", encoding="utf-8") as f:
        json.dump({
            "obstacles": st.session_state.obstacles_all,
            "types": st.session_state.obstacles_type,
            "heights": st.session_state.obstacles_height
        }, f)

def load_all():
    try:
        with open("geo_obstacles.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles_all = data.get("obstacles", [])
            st.session_state.obstacles_type = data.get("types", [])
            st.session_state.obstacles_height = data.get("heights", [])
    except:
        st.session_state.obstacles_all = []
        st.session_state.obstacles_type = []
        st.session_state.obstacles_height = []

load_all()
st.set_page_config(page_title="无人机避障系统（安全半径可调）", layout="wide")

# ================== 核心算法（强制关联安全半径） ==================
def calculate_distance(p1, p2):
    """计算两点间直线距离"""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def calculate_shortest_no_overlap_route():
    """核心算法：所有避障逻辑强制使用自定义安全半径"""
    # 优先使用转换后的坐标
    A = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    B = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    
    if not A or not B:
        return [], "⚠️ 请先设置起点A和终点B！"
    
    # ========== 强制读取安全半径（核心） ==========
    drone_safety_radius_m = st.session_state.drone_safety_radius  # 读取滑块值
    safe_distance = drone_safety_radius_m * SAFE_DISTANCE_LNG_LAT_PER_METER  # 米转经纬度
    drone_h = st.session_state.drone_height
    
    final_route = [A]
    avoid_obstacles = []
    
    # 优先使用转换后的障碍物坐标
    obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    
    # 过滤需要避开的障碍物（高度>无人机高度）
    obstacle_polygons = []
    for i, obs_coords in enumerate(obstacles):
        if len(obs_coords) < 3:
            continue
        
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
        
        if obs_h <= drone_h:
            continue
        
        avoid_obstacles.append(obs_type)
        obs_poly = Polygon(obs_coords)
        # ========== 按安全半径外扩障碍物（核心修改） ==========
        # 外扩比例随安全半径变化：安全半径越大，外扩越多
        expand_ratio = 1.0 + (drone_safety_radius_m / 500)  # 50米对应外扩10%
        obs_poly_scaled = scale(obs_poly, xfact=expand_ratio, yfact=expand_ratio, origin='centroid')
        obstacle_polygons.append({
            "poly": obs_poly,
            "scaled_poly": obs_poly_scaled,
            "type": obs_type,
            "height": obs_h,
            "centroid": obs_poly.centroid
        })
    
    if not obstacle_polygons:
        # 无需要避开的障碍物，直接直线飞行
        final_route = [A, B]
        status = (
            f"🟢 直线飞行（无避障）\n"
            f"✈️ 无人机高度：{drone_h}米\n"
            f"🛡️ 安全半径：{drone_safety_radius_m}米\n"
            f"📏 路径长度≈{calculate_distance(A,B)*111:.3f}公里"
        )
        return final_route, status
    
    # ========== 核心避障逻辑（关联安全半径） ==========
    current_point = A
    target_point = B
    visited_obs = set()
    
    while True:
        # 1. 构建当前点到终点的直线
        direct_line = LineString([current_point, target_point])
        collision = False
        collision_obs = None
        
        # 2. 检查是否碰撞障碍物
        for idx, obs in enumerate(obstacle_polygons):
            if idx in visited_obs:
                continue
            if direct_line.intersects(obs["scaled_poly"]):
                collision = True
                collision_obs = obs
                collision_obs_idx = idx
                break
        
        if not collision:
            final_route.append(target_point)
            break
        
        # 3. 计算绕行点（按安全半径偏移）
        obs_poly = collision_obs["scaled_poly"]
        obs_ring = LinearRing(list(obs_poly.exterior.coords))
        line_point, obs_point = nearest_points(direct_line, obs_ring)
        
        # 计算远离障碍物的方向
        centroid = collision_obs["centroid"]
        dx = obs_point.x - centroid.x
        dy = obs_point.y - centroid.y
        dist = math.hypot(dx, dy) or 1
        
        dx /= dist
        dy /= dist
        
        # ========== 按安全半径计算绕行点 ==========
        shortest_avoid_point = (
            obs_point.x + dx * safe_distance,
            obs_point.y + dy * safe_distance
        )
        
        # 二次校验：确保绕行点不在障碍物内
        if obs_poly.contains(Point(shortest_avoid_point)):
            shortest_avoid_point = (
                obs_point.x + dx * safe_distance * 1.5,
                obs_point.y + dy * safe_distance * 1.5
            )
        
        # 4. 添加绕行点
        final_route.append(shortest_avoid_point)
        current_point = shortest_avoid_point
        visited_obs.add(collision_obs_idx)
    
    # ========== 路径优化 ==========
    # 去重
    final_route_clean = []
    seen = set()
    for point in final_route:
        point_tuple = (round(point[0], 8), round(point[1], 8))
        if point_tuple not in seen:
            seen.add(point_tuple)
            final_route_clean.append(point)
    final_route = final_route_clean
    
    # 最终校验（关联安全半径）
    for i in range(len(final_route)-1):
        segment = LineString([final_route[i], final_route[i+1]])
        for obs in obstacle_polygons:
            if segment.intersects(obs["scaled_poly"]):
                mid_point = (
                    (final_route[i][0] + final_route[i+1][0])/2,
                    (final_route[i][1] + final_route[i+1][1])/2
                )
                centroid = obs["centroid"]
                dx = mid_point[0] - centroid.x
                dy = mid_point[1] - centroid.y
                dist = math.hypot(dx, dy) or 1
                dx /= dist
                dy /= dist
                
                # 按安全半径偏移
                new_avoid_point = (
                    mid_point[0] + dx * safe_distance,
                    mid_point[1] + dy * safe_distance
                )
                final_route.insert(i+1, new_avoid_point)
                break
    
    # 计算总路径长度
    total_distance = 0
    for i in range(len(final_route)-1):
        total_distance += calculate_distance(final_route[i], final_route[i+1])
    total_distance_km = total_distance * 111
    
    # ========== 状态信息（强制显示安全半径） ==========
    status = (
        f"🔴 避障飞行（安全半径生效）\n"
        f"✈️ 无人机高度：{drone_h}米\n"
        f"🛡️ 安全半径：{drone_safety_radius_m}米（核心参数）\n"
        f"🚫 避开障碍物：{','.join(avoid_obstacles)}\n"
        f"📏 路径长度≈{total_distance_km:.3f}公里"
    )
    return final_route, status

# ================== 心跳包更新函数 ==================
def update_drone_heartbeat():
    if not st.session_state.heartbeat_running:
        return
    
    now = datetime.now()
    time_diff = (now - st.session_state.drone_heartbeat["last_time"]).total_seconds()
    
    if time_diff >= st.session_state.drone_heartbeat["heartbeat_interval"]:
        st.session_state.drone_heartbeat["heartbeat_seq"] += 1
        
        new_signal = st.session_state.drone_heartbeat["signal_strength"] + random.randint(-2, 2)
        st.session_state.drone_heartbeat["signal_strength"] = max(80, min(100, new_signal))
        
        if st.session_state.drone_heartbeat["battery"] > 0 and random.randint(1, 5) == 3:
            st.session_state.drone_heartbeat["battery"] -= 1
        
        st.session_state.drone_heartbeat["gps_status"] = "正常"
        
        _, route_status = calculate_shortest_no_overlap_route()
        if st.session_state.point_a and st.session_state.point_b:
            st.session_state.drone_heartbeat["flight_status"] = f"避障中（安全半径：{st.session_state.drone_safety_radius}米）"
        else:
            st.session_state.drone_heartbeat["flight_status"] = "待命"
        
        st.session_state.drone_heartbeat["last_time"] = now
        
        log_entry = {
            "时间": now.strftime("%Y-%m-%d %H:%M:%S"),
            "心跳序号": st.session_state.drone_heartbeat["heartbeat_seq"],
            "信号强度": f"{st.session_state.drone_heartbeat['signal_strength']}%",
            "电池电量": f"{st.session_state.drone_heartbeat['battery']}%",
            "GPS状态": st.session_state.drone_heartbeat["gps_status"],
            "飞行状态": st.session_state.drone_heartbeat["flight_status"],
            "安全半径": f"{st.session_state.drone_safety_radius}米"  # 日志中添加安全半径
        }
        st.session_state.heartbeat_log.insert(0, log_entry)
        
        if len(st.session_state.heartbeat_log) > 50:
            st.session_state.heartbeat_log = st.session_state.heartbeat_log[:50]
        
        st.session_state.heartbeat_chart_data["time"].append(now.strftime("%H:%M:%S"))
        st.session_state.heartbeat_chart_data["seq"].append(st.session_state.drone_heartbeat["heartbeat_seq"])
        
        if len(st.session_state.heartbeat_chart_data["time"]) > 20:
            st.session_state.heartbeat_chart_data["time"] = st.session_state.heartbeat_chart_data["time"][-20:]
            st.session_state.heartbeat_chart_data["seq"] = st.session_state.heartbeat_chart_data["seq"][-20:]

# ================== 页面布局（重点：突出安全半径设置） ==================
st.title("✈️ 无人机避障系统 | 安全半径可调节版")
st.markdown("---")

# ========== 侧边栏（强制突出安全半径设置） ==========
with st.sidebar:
    st.subheader("⚙️ 核心参数设置")
    
    # ========== 1. 安全半径设置（置顶+高亮） ==========
    st.markdown("### 🛡️ 无人机避障安全半径（核心）")
    st.markdown("**设置无人机与障碍物的最小安全距离**")
    # 滑块（加大尺寸，突出显示）
    st.session_state.drone_safety_radius = st.slider(
        "安全半径（米）",
        min_value=1, 
        max_value=50, 
        value=st.session_state.drone_safety_radius,
        step=1,
        key="safety_radius_slider",
        help="调节无人机需要远离障碍物的最小距离，数值越大，绕行越远"
    )
    # 实时显示当前值（加粗）
    st.markdown(f"""
    <div style='background-color:#e8f4f8; padding:10px; border-radius:5px;'>
    <strong>当前安全半径：{st.session_state.drone_safety_radius} 米</strong>
    <br>对应经纬度距离：{st.session_state.drone_safety_radius * SAFE_DISTANCE_LNG_LAT_PER_METER:.8f} 度
    </div>
    """, unsafe_allow_html=True)
    st.markdown("---")
    
    # ========== 2. 无人机高度设置 ==========
    st.subheader("✈️ 无人机飞行高度")
    st.session_state.drone_height = st.slider(
        "飞行高度（米，地面以上）",
        min_value=0, max_value=200, value=8, step=1,
        key="drone_height_slider"
    )
    st.caption(f"当前：{st.session_state.drone_height}米")
    st.markdown("---")
    
    # ========== 3. 坐标系选择 ==========
    st.subheader("🗺️ 坐标系切换")
    coord_option = st.radio(
        "选择坐标系统",
        ["WGS84（原始GPS）", "GCJ02（火星坐标系）"],
        index=0 if st.session_state.coord_system == "WGS84（原始GPS）" else 1
    )
    if coord_option != st.session_state.coord_system:
        st.session_state.coord_system = coord_option
        # 转换坐标
        if st.session_state.point_a:
            if coord_option == "GCJ02（火星坐标系）":
                st.session_state.transformed_points["point_a"] = wgs84_to_gcj02(*st.session_state.point_a[::-1])[::-1]
            else:
                st.session_state.transformed_points["point_a"] = gcj02_to_wgs84(*st.session_state.point_a[::-1])[::-1]
        if st.session_state.point_b:
            if coord_option == "GCJ02（火星坐标系）":
                st.session_state.transformed_points["point_b"] = wgs84_to_gcj02(*st.session_state.point_b[::-1])[::-1]
            else:
                st.session_state.transformed_points["point_b"] = gcj02_to_wgs84(*st.session_state.point_b[::-1])[::-1]
        # 转换障碍物坐标
        st.session_state.transformed_points["obstacles"] = []
        for obs in st.session_state.obstacles_all:
            transformed_obs = []
            for point in obs:
                if coord_option == "GCJ02（火星坐标系）":
                    transformed_point = wgs84_to_gcj02(*point[::-1])[::-1]
                else:
                    transformed_point = gcj02_to_wgs84(*point[::-1])[::-1]
                transformed_obs.append(transformed_point)
            st.session_state.transformed_points["obstacles"].append(transformed_obs)
    
    # ========== 4. 障碍物绘制 ==========
    st.markdown("---")
    st.subheader("🌍 障碍物绘制")
    st.warning("⚠️ 障碍物高度>无人机高度时，自动按安全半径绕行！")
    
    drawing_mode = st.radio(
        "绘制模式",
        ["选择起点A", "选择终点B", "绘制障碍物", "取消绘制"],
        key="drawing_mode_radio"
    )
    
    if drawing_mode == "取消绘制":
        st.session_state.drawing_mode = None
        st.session_state.current_points = []
    else:
        st.session_state.drawing_mode = drawing_mode
    
    obstacle_type = st.selectbox(
        "障碍物类型（自动匹配高度）",
        list(REAL_WORLD_HEIGHTS.keys()),
        key="obstacle_type_select"
    )
    
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("✅ 确认添加障碍物"):
            if len(st.session_state.current_points) >= 3:
                st.session_state.obstacles_all.append(st.session_state.current_points)
                st.session_state.obstacles_type.append(obstacle_type)
                st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[obstacle_type])
                st.session_state.current_points = []
                save_all()
                st.success(f"✅ 添加{obstacle_type}成功！高度：{REAL_WORLD_HEIGHTS[obstacle_type]}米")
            else:
                st.error("❌ 障碍物需要至少3个点！")
    with col_btn2:
        if st.button("🗑️ 清空障碍物"):
            st.session_state.obstacles_all = []
            st.session_state.obstacles_type = []
            st.session_state.obstacles_height = []
            st.session_state.transformed_points["obstacles"] = []
            save_all()
            st.success("✅ 已清空所有障碍物！")
    
    # ========== 5. 心跳包控制 ==========
    st.markdown("---")
    st.subheader("❤️ 心跳包控制")
    heartbeat_col1, heartbeat_col2 = st.columns(2)
    with heartbeat_col1:
        if st.button("▶️ 启动心跳"):
            st.session_state.heartbeat_running = True
    with heartbeat_col2:
        if st.button("⏹️ 停止心跳"):
            st.session_state.heartbeat_running = False
    
    heartbeat_interval = st.slider(
        "心跳间隔（秒）",
        min_value=1, max_value=10, value=1, step=1,
        key="heartbeat_interval_slider"
    )
    st.session_state.drone_heartbeat["heartbeat_interval"] = heartbeat_interval

# ========== 主页面 ==========
col1, col2 = st.columns([2, 1])

with col1:
    st.subheader(f"🗺️ 地图操作区 | 当前安全半径：{st.session_state.drone_safety_radius} 米")
    
    # 初始化地图（高德地图）
    center_lat = 32.2330
    center_lng = 118.7490
    m = folium.Map(
        location=[center_lat, center_lng],
        zoom_start=18,
        tiles="https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}",
        attr="高德地图"
    )
    
    # 绘制起点A
    if st.session_state.point_a:
        current_a = st.session_state.transformed_points["point_a"] or st.session_state.point_a
        folium.Marker(
            location=current_a,
            popup=f"起点A {current_a}",
            icon=folium.Icon(color="green", icon="plane")
        ).add_to(m)
    
    # 绘制终点B
    if st.session_state.point_b:
        current_b = st.session_state.transformed_points["point_b"] or st.session_state.point_b
        folium.Marker(
            location=current_b,
            popup=f"终点B {current_b}",
            icon=folium.Icon(color="red", icon="flag")
        ).add_to(m)
    
    # 绘制障碍物
    current_obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    for i, obs in enumerate(current_obstacles):
        if len(obs) >= 3:
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
            # 障碍物弹窗显示安全半径
            popup_text = f"""
            {obs_type}
            高度：{obs_h}米
            安全半径：{st.session_state.drone_safety_radius}米
            """
            folium.Polygon(
                locations=obs,
                popup=popup_text,
                color="orange",
                fill=True,
                fill_color="orange",
                fill_opacity=0.3
            ).add_to(m)
    
    # 绘制临时点
    if st.session_state.current_points:
        folium.PolyLine(
            locations=st.session_state.current_points + [st.session_state.current_points[0]] if len(st.session_state.current_points) > 2 else st.session_state.current_points,
            color="blue",
            dash_array="5, 5"
        ).add_to(m)
    
    # 计算并绘制航线
    route, route_status = calculate_shortest_no_overlap_route()
    if route:
        # 转换航线坐标
        transformed_route = []
        for point in route:
            if st.session_state.coord_system == "GCJ02（火星坐标系）":
                transformed_point = wgs84_to_gcj02(*point[::-1])[::-1]
            else:
                transformed_point = gcj02_to_wgs84(*point[::-1])[::-1]
            transformed_route.append(transformed_point)
        
        # 绘制航线
        folium.PolyLine(
            locations=transformed_route,
            color="blue",
            weight=3,
            popup=route_status
        ).add_to(m)
        
        # 绘制绕行点（标注安全半径）
        for idx, point in enumerate(transformed_route[1:-1]):
            folium.CircleMarker(
                location=point, 
                radius=8, 
                color='blue', 
                fill=True, 
                fill_color='yellow',
                # 弹窗强制显示安全半径
                popup=f"绕行点 {idx+1}\n安全半径：{st.session_state.drone_safety_radius}米\n远离障碍物：{st.session_state.drone_safety_radius}米"
            ).add_to(m)
    
    # 渲染地图
    map_data = st_folium(m, width=None, height=600)
    
    # 处理地图点击
    if map_data.get("last_clicked"):
        lat = map_data["last_clicked"]["lat"]
        lng = map_data["last_clicked"]["lng"]
        clicked_point = (lat, lng)
        
        # 转换为原始坐标
        if st.session_state.coord_system == "GCJ02（火星坐标系）":
            original_point = gcj02_to_wgs84(lng, lat)[::-1]
        else:
            original_point = clicked_point
        
        if st.session_state.drawing_mode == "选择起点A":
            st.session_state.point_a = original_point
            st.session_state.transformed_points["point_a"] = clicked_point
            st.success(f"✅ 已选择起点A：{clicked_point}")
        elif st.session_state.drawing_mode == "选择终点B":
            st.session_state.point_b = original_point
            st.session_state.transformed_points["point_b"] = clicked_point
            st.success(f"✅ 已选择终点B：{clicked_point}")
        elif st.session_state.drawing_mode == "绘制障碍物":
            st.session_state.current_points.append(clicked_point)
            st.info(f"🟡 已添加障碍物点 {len(st.session_state.current_points)}：{clicked_point}")

with col2:
    st.subheader("📊 系统状态面板")
    
    # 心跳包状态
    update_drone_heartbeat()
    st.markdown("### ❤️ 无人机实时状态")
    # 状态卡片（显示安全半径）
    st.markdown(f"""
    <div style='background-color:#f0f8fb; padding:15px; border-radius:8px; margin-bottom:10px;'>
    <strong>核心参数</strong><br>
    🛡️ 安全半径：{st.session_state.drone_safety_radius} 米<br>
    ✈️ 飞行高度：{st.session_state.drone_height} 米<br>
    📍 坐标系：{st.session_state.coord_system}
    </div>
    """, unsafe_allow_html=True)
    
    heartbeat_col1, heartbeat_col2 = st.columns(2)
    with heartbeat_col1:
        st.metric("信号强度", f"{st.session_state.drone_heartbeat['signal_strength']}%")
        st.metric("电池电量", f"{st.session_state.drone_heartbeat['battery']}%")
    with heartbeat_col2:
        st.metric("GPS状态", st.session_state.drone_heartbeat['gps_status'])
        st.metric("飞行状态", st.session_state.drone_heartbeat['flight_status'])
    
    # 航线状态
    st.markdown("### 🛩️ 航线规划状态")
    # 状态文本框（突出安全半径）
    st.markdown(f"""
    <div style='background-color:#f5f5f5; padding:10px; border-radius:5px; border-left:4px solid #2196f3;'>
    {route_status}
    </div>
    """, unsafe_allow_html=True)
    
    # 心跳包日志（包含安全半径）
    st.markdown("### 📜 心跳日志（含安全半径）")
    if st.session_state.heartbeat_log:
        df_log = pd.DataFrame(st.session_state.heartbeat_log[:20])
        st.dataframe(df_log, use_container_width=True)
    else:
        st.info("点击侧边栏「启动心跳」开始记录（含安全半径日志）")
    
    # 心跳趋势图
    st.markdown("### 📈 心跳趋势")
    if st.session_state.heartbeat_chart_data["time"]:
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=st.session_state.heartbeat_chart_data["time"],
            y=st.session_state.heartbeat_chart_data["seq"],
            mode="lines+markers",
            name="心跳序号"
        ))
        fig.update_layout(
            height=200,
            margin=dict(l=0, r=0, t=20, b=0),
            xaxis_title="时间",
            yaxis_title="心跳序号"
        )
        st.plotly_chart(fig, use_container_width=True)
    else:
        st.info("启动心跳后显示趋势图")

st.markdown("---")
st.caption(f"© 2025 无人机避障系统 | 当前安全半径：{st.session_state.drone_safety_radius} 米")