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

# ================== 初始化 ==================
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
# 新增：无人机安全半径初始化
if "drone_safety_radius" not in st.session_state:
    st.session_state.drone_safety_radius = 15  # 默认15米
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
    st.session_state.coord_system = "WGS84（原始GPS）"  # 默认WGS84
if "transformed_points" not in st.session_state:
    st.session_state.transformed_points = {
        "point_a": None,
        "point_b": None,
        "obstacles": []
    }

# ================== 核心配置 ==================
GROUND_HEIGHT = 0  # 地面基准高度
# 修改：安全距离从固定值改为基于安全半径的动态计算（经纬度单位，1米≈0.00001）
def get_safe_distance():
    """根据安全半径计算经纬度单位的安全距离"""
    return st.session_state.drone_safety_radius * 0.00001

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
st.set_page_config(page_title="最短无重叠避障无人机系统", layout="wide")

# ================== 核心优化：最短避障航线算法 ==================
def calculate_distance(p1, p2):
    """计算两点间直线距离（几何最短）"""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def calculate_shortest_no_overlap_route():
    """
    核心优化点：
    1. 先找A到B的直线，判断是否碰撞
    2. 碰撞时计算「最短切线点」，只绕障碍物边缘最短路径
    3. 多次迭代优化，保证整体路径最短且无重叠
    4. 新增：基于安全半径动态调整绕行距离
    """
    # 优先使用转换后的坐标
    A = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    B = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    
    if not A or not B:
        return [], "未设置起点A/终点B（地面基准：0米）"
    
    drone_h = st.session_state.drone_height
    # 新增：获取动态安全距离
    SAFE_DISTANCE = get_safe_distance()
    final_route = [A]
    avoid_obstacles = []
    
    # 优先使用转换后的障碍物坐标
    obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    
    # 过滤需要避开的障碍物（高度>无人机高度），并构建安全多边形
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
        # 修改：基于安全半径动态扩大障碍物多边形（避免重叠）
        scale_factor = 1.0 + (st.session_state.drone_safety_radius / 1000)  # 按安全半径比例扩大
        obs_poly_scaled = scale(obs_poly, xfact=scale_factor, yfact=scale_factor, origin='centroid')
        obstacle_polygons.append({
            "poly": obs_poly,
            "scaled_poly": obs_poly_scaled,
            "type": obs_type,
            "height": obs_h,
            "centroid": obs_poly.centroid
        })
    
    if not obstacle_polygons:
        # 无需要避开的障碍物，直接直线飞行（最短路径）
        final_route = [A, B]
        status = f"🟢 最短直线飞行！无人机高度({drone_h}m) ≥ 所有障碍物高度，直接从A到B（安全半径：{st.session_state.drone_safety_radius}米）"
        return final_route, status
    
    # ========== 核心：最短切线绕行逻辑 ==========
    current_point = A
    target_point = B
    visited_obs = set()  # 避免重复绕同一个障碍物
    
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
            # 无碰撞，添加终点，路径完成
            final_route.append(target_point)
            break
        
        # 3. 有碰撞：计算「最短切线点」（几何最短绕行点）
        obs_poly = collision_obs["scaled_poly"]
        obs_ring = LinearRing(list(obs_poly.exterior.coords))
        
        # 找直线与障碍物的最近点
        line_point, obs_point = nearest_points(direct_line, obs_ring)
        
        # 计算从障碍物中心向外的方向（保证在障碍物外，且路径最短）
        centroid = collision_obs["centroid"]
        dx = obs_point.x - centroid.x
        dy = obs_point.y - centroid.y
        dist = math.hypot(dx, dy) or 1  # 避免除0
        
        # 归一化方向向量
        dx /= dist
        dy /= dist
        
        # 修改：使用动态安全距离计算绕行点
        shortest_avoid_point = (
            obs_point.x + dx * SAFE_DISTANCE,
            obs_point.y + dy * SAFE_DISTANCE
        )
        
        # 二次校验：确保绕行点不在障碍物内
        if obs_poly.contains(Point(shortest_avoid_point)):
            shortest_avoid_point = (
                obs_point.x + dx * SAFE_DISTANCE * 1.5,
                obs_point.y + dy * SAFE_DISTANCE * 1.5
            )
        
        # 4. 添加最短绕行点，更新当前点
        final_route.append(shortest_avoid_point)
        current_point = shortest_avoid_point
        visited_obs.add(collision_obs_idx)
    
    # ========== 路径优化：去重 + 平滑 ==========
    # 去重（保留顺序）
    final_route_clean = []
    seen = set()
    for point in final_route:
        point_tuple = (round(point[0], 8), round(point[1], 8))
        if point_tuple not in seen:
            seen.add(point_tuple)
            final_route_clean.append(point)
    final_route = final_route_clean
    
    # 最终校验：确保所有线段都不与障碍物重叠
    for i in range(len(final_route)-1):
        segment = LineString([final_route[i], final_route[i+1]])
        for obs in obstacle_polygons:
            if segment.intersects(obs["scaled_poly"]):
                # 仅在必要时添加中间点，避免绕远
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
                # 修改：使用动态安全距离
                new_avoid_point = (
                    mid_point[0] + dx * SAFE_DISTANCE,
                    mid_point[1] + dy * SAFE_DISTANCE
                )
                final_route.insert(i+1, new_avoid_point)
                break
    
    # 计算总路径长度（展示最短特性）
    total_distance = 0
    for i in range(len(final_route)-1):
        total_distance += calculate_distance(final_route[i], final_route[i+1])
    total_distance_km = total_distance * 111  # 经纬度距离转公里（近似）
    
    status = f"🔴 最短无重叠绕行！无人机高度({drone_h}m) < 障碍物高度，已避开：{','.join(avoid_obstacles)}，总路径长度≈{total_distance_km:.3f}公里（安全半径：{st.session_state.drone_safety_radius}米，仅贴障碍物边缘最短绕行）"
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
            st.session_state.drone_heartbeat["flight_status"] = "最短绕飞中" if "绕行" in route_status else "直线飞行中"
            st.session_state.drone_heartbeat["speed"] = round(random.uniform(4.0, 6.0), 1)
        else:
            st.session_state.drone_heartbeat["flight_status"] = "待命"
            st.session_state.drone_heartbeat["speed"] = 0.0
        
        st.session_state.drone_heartbeat["last_time"] = now
        
        heartbeat_log_entry = {
            "time": now.strftime("%Y-%m-%d %H:%M:%S"),
            "seq": st.session_state.drone_heartbeat["heartbeat_seq"],
            "signal": st.session_state.drone_heartbeat["signal_strength"],
            "battery": st.session_state.drone_heartbeat["battery"],
            "gps": st.session_state.drone_heartbeat["gps_status"],
            "status": st.session_state.drone_heartbeat["flight_status"]
        }
        st.session_state.heartbeat_log.append(heartbeat_log_entry)
        if len(st.session_state.heartbeat_log) > 50:
            st.session_state.heartbeat_log = st.session_state.heartbeat_log[-50:]
        
        st.session_state.heartbeat_chart_data["time"].append(now.strftime("%H:%M:%S"))
        st.session_state.heartbeat_chart_data["seq"].append(st.session_state.drone_heartbeat["heartbeat_seq"])
        if len(st.session_state.heartbeat_chart_data["time"]) > 30:
            st.session_state.heartbeat_chart_data["time"] = st.session_state.heartbeat_chart_data["time"][-30:]
            st.session_state.heartbeat_chart_data["seq"] = st.session_state.heartbeat_chart_data["seq"][-30:]

# ================== 绘制心跳折线图 ==================
def draw_heartbeat_chart():
    df = pd.DataFrame({
        "时间": st.session_state.heartbeat_chart_data["time"],
        "心跳包序号": st.session_state.heartbeat_chart_data["seq"]
    })
    
    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=df["时间"],
        y=df["心跳包序号"],
        mode="lines+markers",
        line=dict(color="#1E88E5", width=3),
        marker=dict(size=6, color="#1E88E5", symbol="circle"),
        name="心跳包序号"
    ))
    
    fig.update_layout(
        title="心跳包实时曲线",
        title_font=dict(size=18, weight="bold", color="#333"),
        xaxis_title="北京时间",
        yaxis_title="心跳包序号",
        xaxis=dict(
            tickangle=-45,
            tickfont=dict(size=10),
            gridcolor="#EEEEEE"
        ),
        yaxis=dict(
            range=[0, max(st.session_state.heartbeat_chart_data["seq"]) + 5 if st.session_state.heartbeat_chart_data["seq"] else 10],
            tickfont=dict(size=10),
            gridcolor="#EEEEEE"
        ),
        height=450,
        margin=dict(l=30, r=20, t=50, b=80),
        plot_bgcolor="#FFFFFF"
    )
    
    return fig

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("无人机最短无重叠避障系统")
    st.info(f"📌 地面基准高度：{GROUND_HEIGHT}米")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    # 坐标系转换设置
    if page == "航线规划":
        st.markdown("---")
        st.subheader("🌐 坐标系转换")
        st.session_state.coord_system = st.selectbox(
            "目标坐标系",
            ["WGS84（原始GPS）", "GCJ02（火星坐标系）"],
            key="coord_select"
        )
        # 确认转换按键
        if st.button("✅ 确认转换坐标", type="primary"):
            # 转换起点A
            if st.session_state.point_a:
                lat_a, lng_a = st.session_state.point_a
                if st.session_state.coord_system == "GCJ02（火星坐标系）":
                    new_lng, new_lat = wgs84_to_gcj02(lng_a, lat_a)
                else:
                    new_lng, new_lat = gcj02_to_wgs84(lng_a, lat_a)
                st.session_state.transformed_points["point_a"] = [round(new_lat, 6), round(new_lng, 6)]
            else:
                st.session_state.transformed_points["point_a"] = None
            
            # 转换终点B
            if st.session_state.point_b:
                lat_b, lng_b = st.session_state.point_b
                if st.session_state.coord_system == "GCJ02（火星坐标系）":
                    new_lng, new_lat = wgs84_to_gcj02(lng_b, lat_b)
                else:
                    new_lng, new_lat = gcj02_to_wgs84(lng_b, lat_b)
                st.session_state.transformed_points["point_b"] = [round(new_lat, 6), round(new_lng, 6)]
            else:
                st.session_state.transformed_points["point_b"] = None
            
            # 转换障碍物坐标
            transformed_obs = []
            for obs in st.session_state.obstacles_all:
                new_obs = []
                for (lat, lng) in obs:
                    if st.session_state.coord_system == "GCJ02（火星坐标系）":
                        new_lng, new_lat = wgs84_to_gcj02(lng, lat)
                    else:
                        new_lng, new_lat = gcj02_to_wgs84(lng, lat)
                    new_obs.append([round(new_lat, 6), round(new_lng, 6)])
                transformed_obs.append(new_obs)
            st.session_state.transformed_points["obstacles"] = transformed_obs
            
            st.success(f"✅ 坐标已转换为「{st.session_state.coord_system}」！")
        
        # 重置坐标按钮
        if st.button("🔄 重置为原始坐标"):
            st.session_state.transformed_points = {
                "point_a": None,
                "point_b": None,
                "obstacles": []
            }
            st.info("🔧 已重置为原始WGS84坐标！")

    # 无人机高度设置
    st.markdown("---")
    st.subheader("🛸 无人机飞行参数")
    st.session_state.drone_height = st.slider(
        "飞行高度（地面以上/米）",
        min_value=0, max_value=200, value=8, step=1,
        key="drone_height_slider"
    )
    st.caption(f"当前高度：{st.session_state.drone_height}米")
    
    # 新增：无人机安全半径设置
    st.session_state.drone_safety_radius = st.slider(
        "安全半径（与障碍物最小距离/米）",
        min_value=1, max_value=50, value=15, step=1,
        key="drone_safety_radius_slider"
    )
    st.caption(f"当前安全半径：{st.session_state.drone_safety_radius}米")

    # 障碍物圈选
    st.markdown("---")
    st.subheader("🌍 多边形障碍物圈选")
    st.warning(f"⚠️ 高度超标时，航线沿障碍物边缘{st.session_state.drone_safety_radius}米安全距离绕行！")
    
    draw_type = st.selectbox(
        "选择障碍物类型（匹配真实高度）",
        ["无", "自定义障碍物", "普通房屋", "高层楼房", "大树/电线杆", "操场/空地", "桥梁/高架", "塔楼/信号塔"],
        key="draw_type_select"
    )
    
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选") and draw_type != "无":
            st.session_state.drawing_mode = draw_type
            st.session_state.current_points = []
            st.success(f"开始圈选「{draw_type}」（预设高度：{REAL_WORLD_HEIGHTS[draw_type]}米）")
    with col2:
        if st.button("✅ 完成圈选") and st.session_state.drawing_mode:
            if len(st.session_state.current_points) >= 3:
                if st.session_state.current_points[0] != st.session_state.current_points[-1]:
                    st.session_state.current_points.append(st.session_state.current_points[0])
                st.session_state.obstacles_all.append(st.session_state.current_points)
                st.session_state.obstacles_type.append(st.session_state.drawing_mode)
                st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[st.session_state.drawing_mode])
                save_all()
                st.success(f"「{st.session_state.drawing_mode}」添加成功！高度：{REAL_WORLD_HEIGHTS[st.session_state.drawing_mode]}米")
            else:
                st.error("❌ 至少需要3个点形成多边形！")
            st.session_state.drawing_mode = None
            st.session_state.current_points = []

    # 清空按钮
    if st.button("🗑️ 清空所有障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_type = []
        st.session_state.obstacles_height = []
        save_all()
        st.success("✅ 所有障碍物已清空！")

    # 障碍物高度自定义
