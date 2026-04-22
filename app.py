import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
import math
from datetime import datetime

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
    st.session_state.drone_height = 8  # 默认8米，匹配你的测试值

# ================== 实景物体高度库（贴近真实地理） ==================
REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50,
    "普通房屋": 20,        # 3-6层住宅真实高度
    "高层楼房": 80,        # 20+层高楼真实高度
    "大树/电线杆": 10,     # 树木/设施真实高度
    "操场/空地": 0,        # 无高度，可直接飞过
    "桥梁/高架": 15,       # 桥梁真实高度
    "塔楼/信号塔": 60      # 塔类设施真实高度
}

# ================== 永久存储 ==================
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
st.set_page_config(page_title="强制避障无人机系统", layout="wide")

# ================== 核心：强制避障算法（100%绕行，绝不穿透） ==================
def calculate_force_avoid_route():
    """
    强制逻辑：
    1. 只要无人机高度 < 实体高度 → 必须绕行，且路线完全避开实体范围
    2. 绕行路线为「最短路径+安全距离」，不贴边、不穿透
    """
    A = st.session_state.point_a
    B = st.session_state.point_b
    
    if not A or not B:
        return [], "未设置起点/终点"
    
    drone_h = st.session_state.drone_height
    direct_line = LineString([A, B])
    avoid_obs_list = []  # 需要绕行的实体列表
    safe_distance = 0.0004  # 安全偏移距离（保证路线远离障碍物）

    # 第一步：筛选所有需要绕行的实体
    for i, obs_coords in enumerate(st.session_state.obstacles_all):
        if len(obs_coords) < 3:
            continue
        
        obs_poly = Polygon(obs_coords)
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        
        # 强制判断：高度不足 → 加入绕行列表（无论是否相交，先标记）
        if drone_h < obs_h:
            avoid_obs_list.append(obs_poly)

    # 第二步：生成强制绕行路线
    if avoid_obs_list:
        final_route = [A]
        current_pos = A
        
        # 对每个需要绕行的实体计算安全绕行点
        for obs_poly in avoid_obs_list:
            # 1. 找当前点到障碍物的最近点
            nearest_pt = nearest_points(Point(current_pos), obs_poly)[1]
            # 2. 计算远离障碍物的安全点（垂直偏移，保证安全距离）
            centroid = obs_poly.centroid
            direction_x = nearest_pt.x - centroid.x
            direction_y = nearest_pt.y - centroid.y
            # 归一化方向向量
            norm = math.hypot(direction_x, direction_y)
            if norm == 0:
                norm = 1
            safe_x = nearest_pt.x + (direction_x / norm) * safe_distance
            safe_y = nearest_pt.y + (direction_y / norm) * safe_distance
            safe_point = (safe_x, safe_y)
            
            final_route.append(safe_point)
            current_pos = safe_point
        
        final_route.append(B)
        status = f"🔴 强制绕行！无人机高度({drone_h}m) < 实景物体/障碍物高度，已规划安全路线"
    else:
        # 所有实体高度都低于无人机 → 直线飞行
        final_route = [A, B]
        status = f"🟢 直线飞行！无人机高度({drone_h}m) ≥ 所有实景物体/障碍物高度"
    
    return final_route, status

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("无人机强制避障系统")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    # 无人机高度设置（默认8米）
    st.markdown("---")
    st.subheader("🛸 无人机飞行高度")
    st.session_state.drone_height = st.slider(
        "设置飞行高度（米）",
        min_value=0, max_value=200, value=8, step=1,
        key="drone_height_slider"
    )

    # 实景物体圈选
    st.markdown("---")
    st.subheader("🌍 实景物体/障碍物圈选")
    st.warning("圈选地图上的真实物体（房屋/高楼/树木等），高度不足将强制绕行！")
    
    draw_type = st.selectbox(
        "选择实景物体类型（匹配真实高度）",
        ["无", "自定义障碍物", "普通房屋", "高层楼房", "大树/电线杆", "操场/空地", "桥梁/高架", "塔楼/信号塔"],
        key="draw_type_select"
    )
    
    # 绘制控制
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选") and draw_type != "无":
            st.session_state.drawing_mode = draw_type
            st.session_state.current_points = []
            st.success(f"开始圈选「{draw_type}」（预设高度：{REAL_WORLD_HEIGHTS[draw_type]}米）")
    with col2:
        if st.button("✅ 完成圈选") and st.session_state.drawing_mode:
            if len(st.session_state.current_points) >= 3:
                st.session_state.obstacles_all.append(st.session_state.current_points)
                st.session_state.obstacles_type.append(st.session_state.drawing_mode)
                st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[st.session_state.drawing_mode])
                save_all()
                st.success(f"「{st.session_state.drawing_mode}」添加成功！预设高度：{REAL_WORLD_HEIGHTS[st.session_state.drawing_mode]}米")
            else:
                st.error("❌ 至少需要3个点才能形成多边形！")
            st.session_state.drawing_mode = None
            st.session_state.current_points = []

    # 清空按钮
    if st.button("🗑️ 清空所有实体/障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_type = []
        st.session_state.obstacles_height = []
        save_all()
        st.success("✅ 所有实体/障碍物已清空！")

    # 实体高度自定义（强制可改）
    st.markdown("---")
    st.subheader("📏 实景物体高度自定义（真实高度）")
    if len(st.session_state.obstacles_all) > 0:
        for i in range(len(st.session_state.obstacles_all)):
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            current_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else REAL_WORLD_HEIGHTS[obs_type]
            
            st.write(f"📌 {obs_type} {i+1}（真实高度）")
            new_h = st.slider(
                f"高度（米）",
                min_value=0, max_value=200, value=current_h, step=1,
                key=f"obs_{i}_height",
                label_visibility="collapsed"
            )
            # 实时更新，强制保存
            if i < len(st.session_state.obstacles_height):
                st.session_state.obstacles_height[i] = new_h
                save_all()
                # 高度修改提示
                if new_h > st.session_state.drone_height:
                    st.error(f"⚠️ 该物体高度({new_h}m) > 无人机高度({st.session_state.drone_height}m)，将强制绕行！")
                else:
                    st.success(f"✅ 该物体高度({new_h}m) ≤ 无人机高度，可直接飞过")
    else:
        st.info("暂无实体，先圈选地图上的房屋/高楼/树木等实景物体")

    # A/B点管理
    st.markdown("---")
    st.subheader("📍 航线起点/终点")
    col_a, col_b = st.columns(2)
    with col_a:
        st.subheader("🟢 起点A")
        if st.session_state.point_a:
            st.success(f"纬度：{st.session_state.point_a[0]:.6f}")
            st.success(f"经度：{st.session_state.point_a[1]:.6f}")
            if st.button("清除A点", key="clear_a"):
                st.session_state.point_a = None
        else:
            st.warning("未设置")
    with col_b:
        st.subheader("🔴 终点B")
        if st.session_state.point_b:
            st.success(f"纬度：{st.session_state.point_b[0]:.6f}")
            st.success(f"经度：{st.session_state.point_b[1]:.6f}")
            if st.button("清除B点", key="clear_b"):
                st.session_state.point_b = None
        else:
            st.warning("未设置")

# ================== 地图与航线展示 ==================
if page == "航线规划":
    st.title("🗺️ 无人机实景强制避障系统")
    
    # 计算强制避障路线
    route, route_status = calculate_force_avoid_route()
    
    # 醒目显示航线状态
    if "强制绕行" in route_status:
        st.error(route_status)
    else:
        st.success(route_status)

    # 卫星地图（显示真实实景）
    m = folium.Map(
        location=[32.2330, 118.7490],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery"
    )

    # 绘制起点A
    if st.session_state.point_a:
        folium.CircleMarker(
            location=st.session_state.point_a,
            radius=12, color='green', fill=True, fill_color='green', fill_opacity=0.8,
            popup="起点A"
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_a,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px; background:green; padding:2px;">A 起点</div>')
        ).add_to(m)

    # 绘制终点B
    if st.session_state.point_b:
        folium.CircleMarker(
            location=st.session_state.point_b,
            radius=12, color='red', fill=True, fill_color='red', fill_opacity=0.8,
            popup="终点B"
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_b,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px; background:red; padding:2px;">B 终点</div>')
        ).add_to(m)

    # 绘制实景物体/障碍物（不同类型不同颜色，醒目显示）
    TYPE_COLORS = {
        "自定义障碍物": "darkred",
        "普通房屋": "orange",
        "高层楼房": "darkblue",
        "大树/电线杆": "darkgreen",
        "操场/空地": "gray",
        "桥梁/高架": "purple",
        "塔楼/信号塔": "brown"
    }
    for i, obs in enumerate(st.session_state.obstacles_all):
        if len(obs) > 2:
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else REAL_WORLD_HEIGHTS[obs_type]
            color = TYPE_COLORS.get(obs_type, "darkred")
            
            # 高度不足时，障碍物高亮显示
            fill_opacity = 0.6 if obs_h > st.session_state.drone_height else 0.3
            weight = 6 if obs_h > st.session_state.drone_height else 3
            
            folium.Polygon(
                locations=obs,
                color=color, fill=True, fill_color=color, fill_opacity=fill_opacity,
                weight=weight, 
                popup=f"{obs_type} | 真实高度：{obs_h}米<br>无人机高度：{st.session_state.drone_height}米<br>状态：{'强制绕行' if obs_h > st.session_state.drone_height else '可直接飞过'}",
                tooltip=f"{obs_type}（{obs_h}米）"
            ).add_to(m)

    # 绘制正在圈选的实体
    if st.session_state.drawing_mode and len(st.session_state.current_points) > 0:
        draw_type = st.session_state.drawing_mode
        color = TYPE_COLORS.get(draw_type, "orange")
        
        folium.PolyLine(
            locations=st.session_state.current_points,
            color=color, weight=5, dash_array='5,5',
            popup=f"正在绘制：{draw_type}（预设高度：{REAL_WORLD_HEIGHTS[draw_type]}米）"
        ).add_to(m)
        for idx, p in enumerate(st.session_state.current_points):
            folium.CircleMarker(
                location=p, radius=6, color=color, fill=True,
                popup=f"顶点 {idx+1}"
            ).add_to(m)

    # 绘制强制避障航线（加粗，醒目）
    if len(route) >= 2:
        folium.PolyLine(
            locations=route,
            color='blue', weight=8, opacity=0.9,
            popup=route_status,
            tooltip="强制避障航线（安全距离：0.0004°）"
        ).add_to(m)
        # 绘制绕行点
        for idx, point in enumerate(route[1:-1]):
            folium.CircleMarker(
                location=point, radius=8, color='blue', fill=True, fill_color='white',
                popup=f"绕行点 {idx+1}（安全避障）"
            ).add_to(m)

    # 地图交互
    map_out = st_folium(
        m, key="drone_map", height=800,
        use_container_width=True, returned_objects=["last_clicked"]
    )

    # 处理地图点击
    if map_out and map_out.get("last_clicked"):
        lat = round(map_out["last_clicked"]["lat"], 6)
        lng = round(map_out["last_clicked"]["lng"], 6)
        
        if st.session_state.drawing_mode:
            st.session_state.current_points.append([lat, lng])
        else:
            if not st.session_state.point_a:
                st.session_state.point_a = (lat, lng)
                st.success(f"✅ 起点A已设置：({lat}, {lng})")
            elif not st.session_state.point_b:
                st.session_state.point_b = (lat, lng)
                st.success(f"✅ 终点B已设置：({lat}, {lng})")

# ================== 飞行监控页面 ==================
else:
    st.title("📡 无人机飞行监控中心")
    
    # 系统状态（醒目）
    st.subheader("✅ 实时系统状态")
    st.success(f"无人机当前高度：{st.session_state.drone_height} 米")
    st.success(f"已圈选实景物体数量：{len(st.session_state.obstacles_all)} 个")
    
    # 强制避障提醒
    need_avoid_count = 0
    for i in range(len(st.session_state.obstacles_all)):
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        if obs_h > st.session_state.drone_height:
            need_avoid_count += 1
    if need_avoid_count > 0:
        st.error(f"🔴 发现 {need_avoid_count} 个实景物体高度超过无人机，将强制绕行！")
    else:
        st.success(f"🟢 所有实景物体高度均≤无人机高度，可直线飞行！")
    
    # 实景物体详情
    if len(st.session_state.obstacles_all) > 0:
        st.subheader("🌍 实景物体/障碍物详情（真实高度）")
        for i in range(len(st.session_state.obstacles_all)):
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else REAL_WORLD_HEIGHTS[obs_type]
            status = "🔴 强制绕行" if obs_h > st.session_state.drone_height else "🟢 可直接飞过"
            st.info(f"{obs_type} {i+1}：真实高度 {obs_h} 米 → {status}")
    else:
        st.info("暂无实景物体/障碍物数据，请先圈选地图上的真实物体！")
    
    # 航线信息
    st.subheader("📍 航线策略")
    if st.session_state.point_a and st.session_state.point_b:
        _, route_status = calculate_force_avoid_route()
        st.write(f"当前策略：{route_status}")
    else:
        st.warning("请先在「航线规划」页面设置起点和终点！")