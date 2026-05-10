import streamlit as st
import folium
from streamlit_folium import st_folium
import math
import time
from datetime import timedelta
import json
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
from shapely.affinity import scale
from shapely.ops import unary_union

# ===================== 核心配置 =====================
# 学校区域经纬度范围（南京某校园，可自行调整）
SCHOOL_LAT_RANGE = (32.2300, 32.2380)
SCHOOL_LNG_RANGE = (118.7450, 118.7550)
# GCJ-02与WGS84坐标系转换参数
PI = math.pi
a = 6378245.0
ee = 0.00669342162296594323

# ===================== 坐标系转换（解决坐标偏移） =====================
def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * PI) + 320 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return ret

def out_of_china(lat, lng):
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
    """GCJ02转WGS84"""
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

# ===================== 初始化Session State =====================
def init_session():
    # 基础点位
    if "point_a" not in st.session_state:
        st.session_state.point_a = None  # GCJ02坐标
    if "point_b" not in st.session_state:
        st.session_state.point_b = None  # GCJ02坐标
    # 障碍物
    if "obstacles" not in st.session_state:
        st.session_state.obstacles = []  # 存储：[(多边形坐标列表, 高度), ...]
    if "drawing_obstacle" not in st.session_state:
        st.session_state.drawing_obstacle = False
    if "temp_obstacle_points" not in st.session_state:
        st.session_state.temp_obstacle_points = []
    # 飞行参数
    if "drone_height" not in st.session_state:
        st.session_state.drone_height = 50  # 无人机飞行高度
    if "safety_radius" not in st.session_state:
        st.session_state.safety_radius = 20  # 安全半径（米）
    if "avoid_mode" not in st.session_state:
        st.session_state.avoid_mode = "最优路径"  # 左/右/最优
    # 飞行状态
    if "flight_path" not in st.session_state:
        st.session_state.flight_path = []
    if "drone_pos" not in st.session_state:
        st.session_state.drone_pos = None
    if "flight_idx" not in st.session_state:
        st.session_state.flight_idx = 0
    if "is_flying" not in st.session_state:
        st.session_state.is_flying = False
    if "flight_start_time" not in st.session_state:
        st.session_state.flight_start_time = None
    if "flight_speed" not in st.session_state:
        st.session_state.flight_speed = 8.5  # m/s
    if "battery" not in st.session_state:
        st.session_state.battery = 100.0

init_session()

# ===================== 航线规划核心算法 =====================
def calculate_distance(lat1, lng1, lat2, lng2):
    """计算两点间距离（米）"""
    R = 6371000
    dLat = math.radians(lat2 - lat1)
    dLng = math.radians(lng2 - lng1)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLng/2) * math.sin(dLng/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def interpolate_path(path, steps=20):
    """平滑轨迹插值"""
    smooth = []
    for i in range(len(path)-1):
        lat1, lng1 = path[i]
        lat2, lng2 = path[i+1]
        for s in range(steps):
            f = s / steps
            lat = lat1 + (lat2 - lat1) * f
            lng = lng1 + (lng2 - lng1) * f
            smooth.append((lat, lng))
    return smooth

def calculate_optimal_route():
    """最优路径规划（最短航程+绕障）"""
    A = st.session_state.point_a
    B = st.session_state.point_b
    if not A or not B:
        return []
    
    drone_h = st.session_state.drone_height
    safety_r = st.session_state.safety_radius
    avoid_mode = st.session_state.avoid_mode
    OFFSET = safety_r / 12000.0  # 经纬度偏移系数
    
    # 构建障碍物多边形（带安全半径）
    obstacle_polygons = []
    for obs_points, obs_h in st.session_state.obstacles:
        if obs_h <= drone_h or len(obs_points) < 3:
            continue
        try:
            poly = Polygon(obs_points)
            # 扩大安全半径
            safe_poly = scale(poly, 1 + safety_r/1600, 1 + safety_r/1600, origin='centroid')
            obstacle_polygons.append(safe_poly)
        except:
            continue
    
    if not obstacle_polygons:
        return [A, B]
    
    # 合并所有障碍物
    all_obstacles = unary_union(obstacle_polygons)
    route = [A]
    current_point = A
    
    while calculate_distance(current_point[0], current_point[1], B[0], B[1]) > 10:
        line = LineString([current_point, B])
        if not line.intersects(all_obstacles):
            route.append(B)
            break
        
        # 找到交点和绕飞点
        intersection = line.intersection(all_obstacles)
        nearest_pt = nearest_points(line, all_obstacles.boundary)[0]
        px, py = nearest_pt.x, nearest_pt.y
        
        # 计算绕飞方向
        cx, cy = all_obstacles.centroid.x, all_obstacles.centroid.y
        dx = px - cx
        dy = py - cy
        dist = math.hypot(dx, dy) or 1
        dx /= dist
        dy /= dist
        
        if avoid_mode == "向左绕飞":
            wx, wy = -dy, dx
        elif avoid_mode == "向右绕飞":
            wx, wy = dy, -dx
        else:  # 最优路径
            # 计算左右绕飞距离，选短的
            left_pt = (px - dy * OFFSET, py + dx * OFFSET)
            right_pt = (px + dy * OFFSET, py - dx * OFFSET)
            left_dist = calculate_distance(left_pt[0], left_pt[1], B[0], B[1])
            right_dist = calculate_distance(right_pt[0], right_pt[1], B[0], B[1])
            if left_dist < right_dist:
                wx, wy = -dy, dx
            else:
                wx, wy = dy, -dx
        
        # 添加绕飞点
        waypoint = (px + wx * OFFSET, py + wy * OFFSET)
        route.append(waypoint)
        current_point = waypoint
    
    return route

# ===================== 界面构建 =====================
st.set_page_config(layout="wide", page_title="无人机航线规划与监控系统")

# 侧边栏导航
page = st.sidebar.radio("功能界面", ["航线规划", "飞行监控"])

if page == "航线规划":
    # ===================== 航线规划界面 =====================
    st.title("✈️ 无人机航线规划系统")
    
    # 布局：左侧控制面板，右侧地图
    col1, col2 = st.columns([1, 3])
    
    with col1:
        st.subheader("📍 坐标点设置（GCJ-02坐标系）")
        
        # 起点A设置
        st.markdown("### 起点 A")
        a_lat = st.number_input(
            "纬度", 
            min_value=SCHOOL_LAT_RANGE[0], 
            max_value=SCHOOL_LAT_RANGE[1], 
            value=32.2322, 
            step=0.0001,
            key="a_lat"
        )
        a_lng = st.number_input(
            "经度", 
            min_value=SCHOOL_LNG_RANGE[0], 
            max_value=SCHOOL_LNG_RANGE[1], 
            value=118.7490, 
            step=0.0001,
            key="a_lng"
        )
        if st.button("✅ 设置 A 点", key="set_a"):
            st.session_state.point_a = (a_lat, a_lng)
            st.success("起点A设置成功！")
        
        # 终点B设置
        st.markdown("### 终点 B")
        b_lat = st.number_input(
            "纬度", 
            min_value=SCHOOL_LAT_RANGE[0], 
            max_value=SCHOOL_LAT_RANGE[1], 
            value=32.2343, 
            step=0.0001,
            key="b_lat"
        )
        b_lng = st.number_input(
            "经度", 
            min_value=SCHOOL_LNG_RANGE[0], 
            max_value=SCHOOL_LNG_RANGE[1], 
            value=118.7490, 
            step=0.0001,
            key="b_lng"
        )
        if st.button("✅ 设置 B 点", key="set_b"):
            st.session_state.point_b = (b_lat, b_lng)
            st.success("终点B设置成功！")
        
        st.divider()
        
        # 飞行参数设置
        st.subheader("🛸 飞行参数")
        st.session_state.drone_height = st.slider(
            "无人机飞行高度 (m)", 
            min_value=0, 
            max_value=200, 
            value=st.session_state.drone_height,
            key="drone_h"
        )
        st.session_state.safety_radius = st.slider(
            "安全半径 (m)", 
            min_value=5, 
            max_value=50, 
            value=st.session_state.safety_radius,
            key="safety_r"
        )
        st.session_state.avoid_mode = st.selectbox(
            "绕飞模式",
            ["最优路径", "向左绕飞", "向右绕飞"],
            key="avoid_mode_sel"
        )
        
        st.divider()
        
        # 障碍物管理
        st.subheader("🌍 障碍物管理")
        obs_height = st.slider(
            "障碍物高度 (m)", 
            min_value=0, 
            max_value=100, 
            value=30,
            key="obs_h"
        )
        
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("🟢 开始圈选障碍物"):
                st.session_state.drawing_obstacle = True
                st.session_state.temp_obstacle_points = []
                st.info("请在地图上点击绘制多边形，至少3个点")
        with col_btn2:
            if st.button("✅ 完成圈选"):
                if len(st.session_state.temp_obstacle_points) >= 3:
                    # 闭合多边形
                    temp_pts = st.session_state.temp_obstacle_points
                    temp_pts.append(temp_pts[0])
                    st.session_state.obstacles.append((temp_pts, obs_height))
                    st.session_state.drawing_obstacle = False
                    st.success(f"障碍物添加成功！当前总数：{len(st.session_state.obstacles)}")
        
        # 障碍物批量操作
        col_btn3, col_btn4 = st.columns(2)
        with col_btn3:
            if st.button("🗑️ 清空所有障碍物"):
                st.session_state.obstacles = []
                st.success("所有障碍物已清空！")
        with col_btn4:
            if st.button("📝 导出障碍物数据"):
                obs_data = json.dumps(st.session_state.obstacles)
                st.download_button(
                    label="下载障碍物数据",
                    data=obs_data,
                    file_name="obstacles.json",
                    mime="application/json"
                )
        
        st.divider()
        
        # 航线生成与飞行控制
        st.subheader("✈️ 航线控制")
        if st.button("📊 生成航线"):
            route = calculate_optimal_route()
            st.session_state.flight_path = interpolate_path(route)
            st.success(f"航线生成成功！共 {len(route)} 个航点，{len(st.session_state.flight_path)} 个插值点")
        
        col_flight1, col_flight2 = st.columns(2)
        with col_flight1:
            if st.button("▶️ 开始飞行"):
                if st.session_state.flight_path:
                    st.session_state.is_flying = True
                    st.session_state.flight_idx = 0
                    st.session_state.drone_pos = st.session_state.flight_path[0]
                    st.session_state.flight_start_time = time.time()
                    st.session_state.battery = 100.0
                    st.success("无人机已起飞！")
                else:
                    st.error("请先生成航线！")
        with col_flight2:
            if st.button("⏹️ 停止飞行"):
                st.session_state.is_flying = False
                st.session_state.drone_pos = None
                st.success("无人机已停止飞行！")
    
    with col2:
        # 地图渲染（3D卫星地图）
        st.subheader("🗺️ 校园卫星地图（GCJ-02坐标系）")
        
        # 确定地图中心
        if st.session_state.point_a:
            center = st.session_state.point_a
        else:
            center = [(SCHOOL_LAT_RANGE[0]+SCHOOL_LAT_RANGE[1])/2, 
                      (SCHOOL_LNG_RANGE[0]+SCHOOL_LNG_RANGE[1])/2]
        
        # 创建3D卫星地图（OpenStreetMap + ESRI卫星图层）
        m = folium.Map(
            location=center,
            zoom_start=18,
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Esri World Imagery (3D)",
            control_scale=True
        )
        
        # 添加OpenStreetMap图层切换
        folium.TileLayer('openstreetmap', name='普通地图').add_to(m)
        folium.LayerControl().add_to(m)
        
        # 绘制起点A
        if st.session_state.point_a:
            folium.Marker(
                location=st.session_state.point_a,
                icon=folium.Icon(color="green", icon="play", prefix="fa"),
                popup=f"起点A\nGCJ-02坐标：{st.session_state.point_a}",
                draggable=True
            ).add_to(m)
        
        # 绘制终点B
        if st.session_state.point_b:
            folium.Marker(
                location=st.session_state.point_b,
                icon=folium.Icon(color="red", icon="flag", prefix="fa"),
                popup=f"终点B\nGCJ-02坐标：{st.session_state.point_b}",
                draggable=True
            ).add_to(m)
        
        # 绘制障碍物
        for i, (obs_pts, obs_h) in enumerate(st.session_state.obstacles):
            folium.Polygon(
                locations=obs_pts,
                color="red",
                fill=True,
                fill_color="red",
                fill_opacity=0.5,
                weight=2,
                popup=f"障碍物 {i+1}\n高度：{obs_h}m\n安全半径：{st.session_state.safety_radius}m"
            ).add_to(m)
        
        # 绘制航线
        if st.session_state.flight_path:
            # 主航线（绿色）
            folium.PolyLine(
                locations=st.session_state.flight_path,
                color="green",
                weight=4,
                opacity=0.8,
                popup=f"规划航线\n总长度：{calculate_distance(st.session_state.flight_path[0][0], st.session_state.flight_path[0][1], st.session_state.flight_path[-1][0], st.session_state.flight_path[-1][1]):.1f}m"
            ).add_to(m)
            # 航点标记
            for i, pt in enumerate(st.session_state.flight_path[::20]):
                folium.CircleMarker(
                    location=pt,
                    radius=3,
                    color="white",
                    fill=True,
                    fill_color="green",
                    popup=f"航点 {i+1}"
                ).add_to(m)
        
        # 绘制无人机
        if st.session_state.drone_pos:
            # 计算机头朝向
            heading = 0
            if st.session_state.flight_idx < len(st.session_state.flight_path)-1:
                lat1, lng1 = st.session_state.flight_path[st.session_state.flight_idx]
                lat2, lng2 = st.session_state.flight_path[st.session_state.flight_idx+1]
                dx = lng2 - lng1
                dy = lat2 - lat1
                heading = math.degrees(math.atan2(dx, dy))
            
            # 旋转的无人机图标
            icon_html = f'''
            <div style="transform: rotate({heading}deg); font-size:28px; color:blue;">✈️</div>
            '''
            folium.Marker(
                location=st.session_state.drone_pos,
                icon=folium.DivIcon(html=icon_html),
                popup=f"无人机位置\n高度：{st.session_state.drone_height}m\n速度：{st.session_state.flight_speed}m/s"
            ).add_to(m)
        
        # 渲染地图（固定容器，防止闪烁）
        map_container = st.empty()
        with map_container:
            map_output = st_folium(
                m, 
                height=700, 
                key="planning_map", 
                returned_objects=["last_clicked"]
            )
        
        # 处理地图点击事件（圈选障碍物）
        if map_output and map_output.get("last_clicked"):
            lat = map_output["last_clicked"]["lat"]
            lng = map_output["last_clicked"]["lng"]
            
            # 验证是否在校园内
            if (SCHOOL_LAT_RANGE[0] <= lat <= SCHOOL_LAT_RANGE[1] and
                SCHOOL_LNG_RANGE[0] <= lng <= SCHOOL_LNG_RANGE[1]):
                if st.session_state.drawing_obstacle:
                    st.session_state.temp_obstacle_points.append((lat, lng))
                    st.info(f"已添加点 {len(st.session_state.temp_obstacle_points)}：({lat:.6f}, {lng:.6f})")
            else:
                st.warning("请在校园范围内选择点位！")

else:
    # ===================== 飞行监控界面 =====================
    st.title("📡 无人机飞行实时监控系统")
    
    # 顶部控制按钮
    col_ctrl1, col_ctrl2, col_ctrl3, col_ctrl4 = st.columns(4)
    with col_ctrl1:
        if st.button("▶️ 开始任务", type="primary"):
            if st.session_state.flight_path:
                st.session_state.is_flying = True
                st.session_state.flight_idx = 0
                st.session_state.drone_pos = st.session_state.flight_path[0]
                st.session_state.flight_start_time = time.time()
                st.session_state.battery = 100.0
    with col_ctrl2:
        if st.button("⏸️ 暂停任务"):
            st.session_state.is_flying = False
    with col_ctrl3:
        if st.button("⏹️ 停止任务"):
            st.session_state.is_flying = False
            st.session_state.drone_pos = None
    with col_ctrl4:
        if st.button("🔄 重置任务"):
            st.session_state.is_flying = False
            st.session_state.flight_idx = 0
            st.session_state.drone_pos = None
            st.session_state.flight_start_time = None
            st.session_state.battery = 100.0
    
    # 实时监控数据面板
    st.subheader("📊 飞行数据监控")
    col_data1, col_data2, col_data3, col_data4, col_data5, col_data6 = st.columns(6)
    
    # 计算监控数据
    total_waypoints = len(st.session_state.flight_path) if st.session_state.flight_path else 0
    current_waypoint = st.session_state.flight_idx + 1 if st.session_state.flight_idx else 0
    flight_progress = (current_waypoint / total_waypoints * 100) if total_waypoints > 0 else 0
    
    # 已用时间
    elapsed_time = 0
    if st.session_state.flight_start_time and st.session_state.is_flying:
        elapsed_time = time.time() - st.session_state.flight_start_time
    elapsed_str = str(timedelta(seconds=int(elapsed_time)))
    
    # 剩余距离
    remaining_distance = 0
    if st.session_state.drone_pos and st.session_state.flight_path:
        if st.session_state.flight_idx < len(st.session_state.flight_path)-1:
            remaining_points = st.session_state.flight_path[st.session_state.flight_idx:]
            remaining_distance = 0
            for i in range(len(remaining_points)-1):
                lat1, lng1 = remaining_points[i]
                lat2, lng2 = remaining_points[i+1]
                remaining_distance += calculate_distance(lat1, lng1, lat2, lng2)
    
    # 预计到达时间
    eta_seconds = remaining_distance / st.session_state.flight_speed if st.session_state.flight_speed > 0 else 0
    eta_str = str(timedelta(seconds=int(eta_seconds)))
    
    # 电量模拟（每秒消耗0.1%）
    if st.session_state.is_flying and elapsed_time > 0:
        st.session_state.battery = max(0, 100 - (elapsed_time * 0.1))
    
    with col_data1:
        st.metric("当前航点", f"{current_waypoint}/{total_waypoints}")
    with col_data2:
        st.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
    with col_data3:
        st.metric("已用时间", elapsed_str)
    with col_data4:
        st.metric("剩余距离", f"{remaining_distance:.1f} m")
    with col_data5:
        st.metric("预计到达", eta_str if eta_seconds > 0 else "已到达")
    with col_data6:
        st.metric("电量模拟", f"{st.session_state.battery:.1f}%")
    
    # 任务进度条
    st.progress(flight_progress / 100, text=f"任务进度：{flight_progress:.1f}%")
    
    # 布局：左侧实时地图，右侧通信链路
    col_map, col_link = st.columns([3, 2])
    
    with col_map:
        st.subheader("🗺️ 实时飞行地图")
        # 渲染实时地图
        if st.session_state.point_a:
            center = st.session_state.point_a
        else:
            center = [(SCHOOL_LAT_RANGE[0]+SCHOOL_LAT_RANGE[1])/2, 
                      (SCHOOL_LNG_RANGE[0]+SCHOOL_LNG_RANGE[1])/2]
        
        m = folium.Map(
            location=center,
            zoom_start=18,
            tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
            attr="Esri World Imagery (3D)"
        )
        
        # 绘制航线和无人机
        if st.session_state.flight_path:
            folium.PolyLine(
                locations=st.session_state.flight_path,
                color="green",
                weight=4,
                opacity=0.8
            ).add_to(m)
        
        if st.session_state.drone_pos:
            folium.Marker(
                location=st.session_state.drone_pos,
                icon=folium.Icon(color="blue", icon="plane", prefix="fa"),
                popup=f"无人机实时位置\n高度：{st.session_state.drone_height}m"
            ).add_to(m)
        
        # 绘制障碍物
        for obs_pts, obs_h in st.session_state.obstacles:
            folium.Polygon(
                locations=obs_pts,
                color="red",
                fill=True,
                fill_color="red",
                fill_opacity=0.5
            ).add_to(m)
        
        st_folium(m, height=500, key="monitor_map")
    
    with col_link:
        st.subheader("📶 通信链路拓扑与数据流")
        
        # 链路状态卡片
        st.markdown("### 链路状态")
        col_link1, col_link2, col_link3 = st.columns(3)
        with col_link1:
            st.success("🟢 GCS 在线")
        with col_link2:
            st.success("🟢 OBC 在线")
        with col_link3:
            st.success("🟢 FCU 在线")
        
        # 链路拓扑图
        st.markdown("### 拓扑结构")
        st.markdown("""
        <div style="display: flex; align-items: center; justify-content: space-around; padding: 20px; background: #f0f2f6; border-radius: 10px;">
            <div style="text-align: center; background: #e6f7ff; padding: 20px; border-radius: 8px;">
                <h4>🖥️ GCS</h4>
                <p>地面站</p>
                <p>192.168.1.100</p>
            </div>
            <div style="color: #1890ff; font-size: 20px;">↔️</div>
            <div style="text-align: center; background: #fff7e6; padding: 20px; border-radius: 8px;">
                <h4>🟣 OBC</h4>
                <p>机载计算机</p>
                <p>Raspberry Pi 4</p>
            </div>
            <div style="color: #1890ff; font-size: 20px;">↔️</div>
            <div style="text-align: center; background: #f9e6ff; padding: 20px; border-radius: 8px;">
                <h4>⚙️ FCU</h4>
                <p>飞控</p>
                <p>PX4/ArduPilot</p>
            </div>
        </div>
        """, unsafe_allow_html=True)
        
        # 链路统计
        st.markdown("### 链路统计")
        st.write("• GCS↔OBC：正常（UDP:14550）")
        st.write("• OBC↔FCU：正常（MAVLink）")
        st.write("• 延迟：~25ms | 丢包率：0.1%")

# ===================== 飞行循环（无闪烁核心） =====================
if st.session_state.is_flying and st.session_state.flight_path:
    total_pts = len(st.session_state.flight_path)
    if st.session_state.flight_idx < total_pts - 1:
        # 平滑更新位置
        st.session_state.flight_idx += 1
        st.session_state.drone_pos = st.session_state.flight_path[st.session_state.flight_idx]
        
        # 控制飞行速度（调整sleep时间）
        time.sleep(0.05)
        
        # 无闪烁刷新
        st.rerun()
    else:
        st.session_state.is_flying = False
        st.success("✅ 无人机已到达目的地！任务完成！")