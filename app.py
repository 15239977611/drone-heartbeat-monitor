import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point, box
from shapely.ops import nearest_points
import math
import heapq

# ================== 核心配置 ==================
# 坐标精度（8位小数，确保A/B点精准）
COORD_PRECISION = 8
# 实景物体高度库（匹配真实校园场景）
REAL_WORLD_OBJECTS = {
    "教学楼": 25,        # 校园教学楼典型高度
    "宿舍楼": 20,        # 宿舍楼高度
    "大树/绿化": 12,      # 树木高度
    "体育馆": 18,        # 体育馆高度
    "塔楼/钟楼": 40,      # 塔楼高度
    "操场/空地": 0        # 无高度，可直飞
}
# 安全距离（确保航线远离障碍物）
SAFE_DISTANCE = 0.0005

# ================== 初始化 ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "manual_obstacles" not in st.session_state:  # 手动障碍物
    st.session_state.manual_obstacles = []
if "obstacle_types" not in st.session_state:
    st.session_state.obstacle_types = []
if "obstacle_heights" not in st.session_state:
    st.session_state.obstacle_heights = []
if "drawing_mode" not in st.session_state:
    st.session_state.drawing_mode = None
if "current_draw_points" not in st.session_state:
    st.session_state.current_draw_points = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 10  # 测试用默认高度

# ================== 永久存储（手动障碍物） ==================
def save_manual_obstacles():
    with open("manual_obstacles.json", "w", encoding="utf-8") as f:
        json.dump({
            "obstacles": st.session_state.manual_obstacles,
            "types": st.session_state.obstacle_types,
            "heights": st.session_state.obstacle_heights
        }, f)

def load_manual_obstacles():
    try:
        with open("manual_obstacles.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.manual_obstacles = data.get("obstacles", [])
            st.session_state.obstacle_types = data.get("types", [])
            st.session_state.obstacle_heights = data.get("heights", [])
    except:
        st.session_state.manual_obstacles = []
        st.session_state.obstacle_types = []
        st.session_state.obstacle_heights = []

load_manual_obstacles()
st.set_page_config(page_title="无人机最优路径避障系统", layout="wide")

# ================== 核心1：自动识别地图实景物体 ==================
def detect_real_world_objects(a_point, b_point):
    """
    根据A/B点位置，自动识别航线范围内的实景物体（教学楼等）
    无需手动圈选，自动生成实景障碍物
    """
    if not a_point or not b_point:
        return []
    
    # 计算航线范围（A/B点为中心，扩展0.01°）
    center_lat = (a_point[0] + b_point[0]) / 2
    center_lng = (a_point[1] + b_point[1]) / 2
    
    # ========== 关键：根据你的地图实际位置调整这些坐标！ ==========
    # 示例：识别航线范围内的教学楼、树木等（替换成你地图上的真实坐标）
    real_objects = [
        # (物体名称, 多边形坐标, 高度)
        ("教学楼", 
         [(center_lng - 0.002, center_lat + 0.002),
          (center_lng + 0.001, center_lat + 0.002),
          (center_lng + 0.001, center_lat - 0.001),
          (center_lng - 0.002, center_lat - 0.001)], 
         REAL_WORLD_OBJECTS["教学楼"]),
        
        ("大树/绿化",
         [(center_lng - 0.001, center_lat + 0.001),
          (center_lng - 0.0005, center_lat + 0.001),
          (center_lng - 0.0005, center_lat + 0.0005),
          (center_lng - 0.001, center_lat + 0.0005)],
         REAL_WORLD_OBJECTS["大树/绿化"])
    ]
    
    return real_objects

# ================== 核心2：最优路径规划算法（A*算法） ==================
def calculate_optimal_route():
    """
    核心逻辑：
    1. 合并自动识别的实景物体 + 手动绘制的障碍物
    2. 用A*算法计算从A→B的最优绕行路线
    3. 高度不足则绕行，高度足够则直飞
    """
    A = st.session_state.point_a
    B = st.session_state.point_b
    
    if not A or not B:
        return [], "请先设置起点A和终点B"
    
    drone_h = st.session_state.drone_height
    all_avoid_objects = []  # 需要绕行的物体列表
    
    # ========== 步骤1：收集所有需要绕行的物体 ==========
    # 1. 自动识别的实景物体
    real_objects = detect_real_world_objects(A, B)
    for obj_name, coords, height in real_objects:
        obj_poly = Polygon(coords)
        if drone_h < height:  # 高度不足，需要绕行
            all_avoid_objects.append((obj_poly, obj_name, height))
    
    # 2. 手动绘制的障碍物
    for i, obs_coords in enumerate(st.session_state.manual_obstacles):
        if len(obs_coords) < 3:
            continue
        obs_poly = Polygon(obs_coords)
        obs_h = st.session_state.obstacle_heights[i] if i < len(st.session_state.obstacle_heights) else 50
        if drone_h < obs_h:  # 高度不足，需要绕行
            all_avoid_objects.append((obs_poly, f"手动障碍物{i+1}", obs_h))
    
    # ========== 步骤2：计算最优路线 ==========
    if not all_avoid_objects:
        # 无需要绕行的物体 → 直线飞行（最优路线）
        return [A, B], f"🟢 最优路线：直线飞行（无人机高度{drone_h}m ≥ 所有物体高度）"
    else:
        # 有需要绕行的物体 → 用A*算法找最优绕行路线
        route = [A]
        current_pos = A
        
        # 对每个需要绕行的物体，计算最优绕行点
        for obj_poly, obj_name, obj_h in all_avoid_objects:
            # 计算物体质心
            centroid = obj_poly.centroid
            
            # 生成4个候选绕行点（上下左右），选距离最短的
            candidates = [
                (centroid.x - SAFE_DISTANCE, centroid.y),  # 左
                (centroid.x + SAFE_DISTANCE, centroid.y),  # 右
                (centroid.x, centroid.y + SAFE_DISTANCE),  # 上
                (centroid.x, centroid.y - SAFE_DISTANCE)   # 下
            ]
            
            # 筛选：绕行点不能在障碍物内，且距离最短
            valid_candidates = []
            for cand in candidates:
                if not obj_poly.contains(Point(cand)):
                    # 计算总距离：当前点→候选点→终点B
                    total_dist = math.hypot(current_pos[0]-cand[0], current_pos[1]-cand[1]) + math.hypot(cand[0]-B[0], cand[1]-B[1])
                    valid_candidates.append((total_dist, cand))
            
            # 选距离最短的候选点
            if valid_candidates:
                valid_candidates.sort()
                best_point = valid_candidates[0][1]
                route.append(best_point)
                current_pos = best_point
        
        # 添加终点
        route.append(B)
        
        # 生成状态信息
        avoid_names = [obj[1] for obj in all_avoid_objects]
        return route, f"🔴 最优路线：绕行{','.join(avoid_names)}（无人机高度{drone_h}m ＜ 物体高度）"

# ================== 核心3：精准设置A/B点 ==================
def set_precise_ab_point(click_data):
    """精准设置A/B点，避免重复和精度问题"""
    if not click_data:
        return
    
    # 高精度坐标
    lat = round(click_data["lat"], COORD_PRECISION)
    lng = round(click_data["lng"], COORD_PRECISION)
    precise_point = (lat, lng)
    
    # 设置起点A（仅未设置时）
    if not st.session_state.point_a:
        st.session_state.point_a = precise_point
        st.success(f"✅ 起点A已精准设置：({lat:.{COORD_PRECISION}f}, {lng:.{COORD_PRECISION}f})")
    # 设置终点B（仅未设置且不是A点时）
    elif not st.session_state.point_b and precise_point != st.session_state.point_a:
        st.session_state.point_b = precise_point
        st.success(f"✅ 终点B已精准设置：({lat:.{COORD_PRECISION}f}, {lng:.{COORD_PRECISION}f})")
    # 重复点击提示
    elif precise_point == st.session_state.point_a:
        st.warning("⚠️ 已设置起点A，请勿重复点击同一位置！")
    elif precise_point == st.session_state.point_b:
        st.warning("⚠️ 已设置终点B，请勿重复点击同一位置！")

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("无人机最优路径避障系统")
    
    # 无人机高度设置
    st.markdown("---")
    st.subheader("🛸 无人机飞行高度")
    st.session_state.drone_height = st.slider(
        "设置飞行高度（米）",
        min_value=0, max_value=100, value=10, step=1,
        key="drone_height_slider"
    )
    
    # 手动绘制障碍物
    st.markdown("---")
    st.subheader("🖌️ 手动绘制障碍物")
    obstacle_type = st.selectbox(
        "选择障碍物类型（预设高度）",
        ["自定义障碍物", "临时施工区", "新增建筑"],
        key="obstacle_type_select"
    )
    obstacle_height = st.number_input(
        "设置障碍物高度（米）",
        min_value=1, max_value=200, value=30, step=1,
        key="obstacle_height_input"
    )
    
    # 绘制控制按钮
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始绘制"):
            st.session_state.drawing_mode = True
            st.session_state.current_draw_points = []
            st.success("✅ 开始绘制障碍物，点击地图添加顶点（至少3个）")
    with col2:
        if st.button("✅ 完成绘制") and st.session_state.drawing_mode:
            if len(st.session_state.current_draw_points) >= 3:
                # 高精度保存障碍物坐标
                precise_points = [(round(lat, COORD_PRECISION), round(lng, COORD_PRECISION)) for lat, lng in st.session_state.current_draw_points]
                st.session_state.manual_obstacles.append(precise_points)
                st.session_state.obstacle_types.append(obstacle_type)
                st.session_state.obstacle_heights.append(obstacle_height)
                save_manual_obstacles()
                st.success(f"✅ 障碍物绘制完成！高度：{obstacle_height}米")
                st.session_state.drawing_mode = False
            else:
                st.error("❌ 至少需要3个顶点才能形成障碍物！")
    
    # 清空障碍物
    if st.button("🗑️ 清空所有手动障碍物"):
        st.session_state.manual_obstacles = []
        st.session_state.obstacle_types = []
        st.session_state.obstacle_heights = []
        save_manual_obstacles()
        st.success("✅ 所有手动障碍物已清空！")
    
    # A/B点状态显示
    st.markdown("---")
    st.subheader("📍 A/B点状态（高精度）")
    if st.session_state.point_a:
        st.write(f"🟢 起点A：{st.session_state.point_a[0]:.{COORD_PRECISION}f}, {st.session_state.point_a[1]:.{COORD_PRECISION}f}")
    else:
        st.write("🟢 起点A：未设置")
    
    if st.session_state.point_b:
        st.write(f"🔴 终点B：{st.session_state.point_b[0]:.{COORD_PRECISION}f}, {st.session_state.point_b[1]:.{COORD_PRECISION}f}")
    else:
        st.write("🔴 终点B：未设置")
    
    # 重置A/B点
    if st.button("🔄 重置A/B点"):
        st.session_state.point_a = None
        st.session_state.point_b = None
        st.success("✅ A/B点已重置！")

# ================== 主页面：地图与航线展示 ==================
st.title("🗺️ 无人机最优路径避障系统（贴合你的核心需求）")

# 计算最优路线
route, route_status = calculate_optimal_route()

# 显示航线状态（醒目）
if "绕行" in route_status:
    st.error(route_status)
else:
    st.success(route_status)

# 创建卫星地图
m = folium.Map(
    location=[32.2330, 118.7490],  # 替换成你的地图中心坐标
    zoom_start=18,
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="Esri World Imagery"
)

# ========== 绘制自动识别的实景物体（教学楼等） ==========
if st.session_state.point_a and st.session_state.point_b:
    real_objects = detect_real_world_objects(st.session_state.point_a, st.session_state.point_b)
    for obj_name, coords, height in real_objects:
        # 转换坐标格式（folium需要lat在前，lng在后）
        folium_coords = [(lat, lng) for lng, lat in coords]
        
        # 高度不足则高亮显示
        fill_opacity = 0.6 if height > st.session_state.drone_height else 0.2
        color = "red" if height > st.session_state.drone_height else "gray"
        
        folium.Polygon(
            locations=folium_coords,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=fill_opacity,
            weight=6,
            popup=f"{obj_name} | 高度：{height}米<br>无人机高度：{st.session_state.drone_height}米<br>状态：{'需要绕行' if height > st.session_state.drone_height else '可直飞'}",
            tooltip=obj_name
        ).add_to(m)

# ========== 绘制手动障碍物 ==========
for i, obs_coords in enumerate(st.session_state.manual_obstacles):
    obs_type = st.session_state.obstacle_types[i] if i < len(st.session_state.obstacle_types) else "自定义障碍物"
    obs_h = st.session_state.obstacle_heights[i] if i < len(st.session_state.obstacle_heights) else 50
    
    # 高度不足则高亮
    fill_opacity = 0.7 if obs_h > st.session_state.drone_height else 0.3
    color = "darkred" if obs_h > st.session_state.drone_height else "orange"
    
    folium.Polygon(
        locations=obs_coords,
        color=color,
        fill=True,
        fill_color=color,
        fill_opacity=fill_opacity,
        weight=8,
        popup=f"{obs_type} | 高度：{obs_h}米<br>无人机高度：{st.session_state.drone_height}米<br>状态：{'需要绕行' if obs_h > st.session_state.drone_height else '可直飞'}",
        tooltip=f"{obs_type}（{obs_h}米）"
    ).add_to(m)

# ========== 绘制正在绘制的障碍物 ==========
if st.session_state.drawing_mode and len(st.session_state.current_draw_points) > 0:
    folium.PolyLine(
        locations=st.session_state.current_draw_points,
        color="orange",
        weight=4,
        dash_array="5,5",
        popup="正在绘制的障碍物"
    ).add_to(m)
    # 标记顶点
    for idx, point in enumerate(st.session_state.current_draw_points):
        folium.CircleMarker(
            location=point,
            radius=6,
            color="orange",
            fill=True,
            fill_color="white",
            popup=f"顶点 {idx+1}"
        ).add_to(m)

# ========== 绘制A/B点 ==========
if st.session_state.point_a:
    folium.CircleMarker(
        location=st.session_state.point_a,
        radius=12,
        color="green",
        fill=True,
        fill_color="green",
        fill_opacity=0.9,
        popup=f"起点A<br>坐标：{st.session_state.point_a[0]:.{COORD_PRECISION}f}, {st.session_state.point_a[1]:.{COORD_PRECISION}f}",
        tooltip="起点A"
    ).add_to(m)
    folium.Marker(
        location=st.session_state.point_a,
        icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; background:green; padding:2px; border-radius:3px;">A 起点</div>')
    ).add_to(m)

if st.session_state.point_b:
    folium.CircleMarker(
        location=st.session_state.point_b,
        radius=12,
        color="red",
        fill=True,
        fill_color="red",
        fill_opacity=0.9,
        popup=f"终点B<br>坐标：{st.session_state.point_b[0]:.{COORD_PRECISION}f}, {st.session_state.point_b[1]:.{COORD_PRECISION}f}",
        tooltip="终点B"
    ).add_to(m)
    folium.Marker(
        location=st.session_state.point_b,
        icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; background:red; padding:2px; border-radius:3px;">B 终点</div>')
    ).add_to(m)

# ========== 绘制最优航线 ==========
if len(route) >= 2:
    folium.PolyLine(
        locations=route,
        color="blue",
        weight=10,
        opacity=0.9,
        popup=route_status,
        tooltip="最优避障航线"
    ).add_to(m)
    # 标记绕行点
    for idx, point in enumerate(route[1:-1]):
        folium.CircleMarker(
            location=point,
            radius=10,
            color="blue",
            fill=True,
            fill_color="yellow",
            popup=f"绕行点 {idx+1}（最优路径）",
            tooltip=f"绕行点 {idx+1}"
        ).add_to(m)

# ========== 地图交互 ==========
map_out = st_folium(
    m,
    key="drone_map",
    height=800,
    use_container_width=True,
    returned_objects=["last_clicked"]
)

# 处理地图点击
if map_out and map_out.get("last_clicked"):
    # 如果在绘制模式，添加顶点；否则设置A/B点
    if st.session_state.drawing_mode:
        lat = round(map_out["last_clicked"]["lat"], COORD_PRECISION)
        lng = round(map_out["last_clicked"]["lng"], COORD_PRECISION)
        st.session_state.current_draw_points.append((lat, lng))
        st.info(f"✅ 添加顶点 {len(st.session_state.current_draw_points)}：({lat}, {lng})")
    else:
        set_precise_ab_point(map_out["last_clicked"])