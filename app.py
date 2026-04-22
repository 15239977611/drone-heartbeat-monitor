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
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "obs_heights" not in st.session_state:
    st.session_state.obs_heights = []
if "drawing_obstacle" not in st.session_state:
    st.session_state.drawing_obstacle = False
if "current_points" not in st.session_state:
    st.session_state.current_points = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 50

# ================== 永久存储 ==================
def save_all():
    with open("obstacles.json", "w", encoding="utf-8") as f:
        json.dump({
            "obs": st.session_state.obstacles,
            "heights": st.session_state.obs_heights
        }, f)

def load_all():
    try:
        with open("obstacles.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles = data.get("obs", [])
            st.session_state.obs_heights = data.get("heights", [])
    except:
        st.session_state.obstacles = []
        st.session_state.obs_heights = []

load_all()
st.set_page_config(page_title="无人机智能绕行系统", layout="wide")

# ================== 核心：精准绕行算法 ==================
def calculate_best_route():
    """
    核心逻辑：
    1. 无人机高度 > 障碍物高度 → 直线飞过
    2. 无人机高度 < 障碍物高度 → 绕开障碍物（找最短平滑路径）
    """
    A = st.session_state.point_a
    B = st.session_state.point_b
    
    # 无A/B点时返回空
    if not A or not B:
        return [], "未设置起点/终点"
    
    # 基础直线
    direct_line = LineString([A, B])
    drone_h = st.session_state.drone_height
    need_avoid = False
    avoid_points = []

    # 遍历所有障碍物判断是否需要绕行
    for i, obs_coords in enumerate(st.session_state.obstacles):
        if len(obs_coords) < 3:
            continue
        
        # 创建障碍物多边形
        obs_poly = Polygon(obs_coords)
        obs_h = st.session_state.obs_heights[i] if i < len(st.session_state.obs_heights) else 50
        
        # 判断1：无人机高度足够 → 跳过该障碍物
        if drone_h > obs_h:
            continue
        
        # 判断2：航线穿过障碍物 → 需要绕行
        if obs_poly.intersects(direct_line):
            need_avoid = True
            
            # 计算绕行点（找障碍物外侧最近点，保证最短路径）
            # 1. 找障碍物中心
            centroid = obs_poly.centroid
            # 2. 计算偏移方向（垂直于AB连线，避开障碍物）
            ab_vector = (B[0]-A[0], B[1]-A[1])
            # 垂直向量（左/右偏移，选更近的一侧）
            perp_vector = (-ab_vector[1], ab_vector[0])
            # 归一化偏移量（保证偏移距离固定，不随地图缩放变）
            scale = 0.0003 / math.hypot(*perp_vector) if math.hypot(*perp_vector) > 0 else 0.0003
            offset_x = perp_vector[0] * scale
            offset_y = perp_vector[1] * scale
            
            # 生成绕行点
            avoid_point = (centroid.x + offset_x, centroid.y + offset_y)
            avoid_points.append(avoid_point)
    
    # 生成最终路线
    if need_avoid and avoid_points:
        # 有障碍物需要绕行 → 起点 → 所有绕行点 → 终点
        final_route = [A] + avoid_points + [B]
        status = f"需要绕行！无人机高度({drone_h}m) < 障碍物高度"
    else:
        # 无绕行需求 → 直线飞行
        final_route = [A, B]
        status = f"直线飞行！无人机高度({drone_h}m) ≥ 所有障碍物高度"
    
    return final_route, status

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("无人机航线控制系统")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    # 无人机高度设置
    st.markdown("---")
    st.subheader("🛸 无人机飞行高度")
    st.session_state.drone_height = st.slider(
        "设置飞行高度（米）",
        min_value=0, max_value=200, value=50, step=1,
        key="drone_height_slider"
    )

    # 障碍物绘制
    st.markdown("---")
    st.subheader("🚧 实体障碍物管理")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选障碍物"):
            st.session_state.drawing_obstacle = True
            st.session_state.current_points = []
    with col2:
        if st.button("✅ 完成圈选"):
            if len(st.session_state.current_points) >= 3:
                st.session_state.obstacles.append(st.session_state.current_points)
                st.session_state.obs_heights.append(50)  # 默认高度50米
                save_all()
                st.success("障碍物添加成功！")
            else:
                st.warning("至少需要3个点才能形成多边形！")
            st.session_state.drawing_obstacle = False
            st.session_state.current_points = []

    # 清除所有障碍物
    if st.button("🗑️ 清空所有障碍物"):
        st.session_state.obstacles = []
        st.session_state.obs_heights = []
        save_all()
        st.success("所有障碍物已清空！")

    # 障碍物高度修改（每个独立滑块）
    st.markdown("---")
    st.subheader("📏 障碍物高度设置")
    if len(st.session_state.obstacles) > 0:
        for i in range(len(st.session_state.obstacles)):
            current_h = st.session_state.obs_heights[i] if i < len(st.session_state.obs_heights) else 50
            new_h = st.slider(
                f"障碍物 {i+1} 高度（米）",
                min_value=0, max_value=200, value=current_h, step=1,
                key=f"obs_{i}_height"
            )
            # 实时更新并保存
            if i < len(st.session_state.obs_heights):
                st.session_state.obs_heights[i] = new_h
                save_all()
    else:
        st.info("暂无障碍物，先圈选添加")

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
    st.title("🗺️ 无人机智能航线规划系统")
    
    # 计算最佳路线
    route, route_status = calculate_best_route()
    
    # 显示航线状态
    st.info(f"📊 航线状态：{route_status}")

    # 初始化卫星地图
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
            radius=10, color='green', fill=True, fill_color='green',
            popup="起点A"
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_a,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px;">A 起点</div>')
        ).add_to(m)

    # 绘制终点B
    if st.session_state.point_b:
        folium.CircleMarker(
            location=st.session_state.point_b,
            radius=10, color='red', fill=True, fill_color='red',
            popup="终点B"
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_b,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px;">B 终点</div>')
        ).add_to(m)

    # 绘制所有障碍物（标注高度）
    for i, obs in enumerate(st.session_state.obstacles):
        if len(obs) > 2:
            h = st.session_state.obs_heights[i] if i < len(st.session_state.obs_heights) else 50
            folium.Polygon(
                locations=obs,
                color='darkred', fill=True, fill_color='red', fill_opacity=0.5,
                weight=4, popup=f"障碍物 {i+1} | 高度：{h}米",
                tooltip=f"障碍物 {i+1}（{h}米）"
            ).add_to(m)

    # 绘制正在圈选的障碍物
    if st.session_state.drawing_obstacle and len(st.session_state.current_points) > 0:
        folium.PolyLine(
            locations=st.session_state.current_points,
            color='orange', weight=4, dash_array='5,5',
            popup="正在绘制的障碍物"
        ).add_to(m)
        # 绘制每个顶点
        for idx, p in enumerate(st.session_state.current_points):
            folium.CircleMarker(
                location=p, radius=5, color='orange', fill=True,
                popup=f"顶点 {idx+1}"
            ).add_to(m)

    # 绘制最终航线（蓝色高亮）
    if len(route) >= 2:
        folium.PolyLine(
            locations=route,
            color='blue', weight=6, opacity=0.9,
            popup=route_status,
            tooltip="最佳航线"
        ).add_to(m)

    # 地图交互
    map_out = st_folium(
        m, key="drone_map", height=750,
        use_container_width=True, returned_objects=["last_clicked"]
    )

    # 处理地图点击（设A/B点 或 绘制障碍物）
    if map_out and map_out.get("last_clicked"):
        lat = round(map_out["last_clicked"]["lat"], 6)
        lng = round(map_out["last_clicked"]["lng"], 6)
        
        if st.session_state.drawing_obstacle:
            # 绘制障碍物：添加顶点
            st.session_state.current_points.append([lat, lng])
        else:
            # 设A/B点：先设A，再设B
            if not st.session_state.point_a:
                st.session_state.point_a = (lat, lng)
                st.success(f"起点A已设置：({lat}, {lng})")
            elif not st.session_state.point_b:
                st.session_state.point_b = (lat, lng)
                st.success(f"终点B已设置：({lat}, {lng})")

# ================== 飞行监控页面 ==================
else:
    st.title("📡 无人机飞行监控中心")
    
    # 实时状态
    st.subheader("✅ 系统状态")
    st.success(f"无人机当前高度：{st.session_state.drone_height} 米")
    st.success(f"已设置障碍物数量：{len(st.session_state.obstacles)} 个")
    
    # 障碍物列表
    if len(st.session_state.obstacles) > 0:
        st.subheader("🚧 障碍物详情")
        for i in range(len(st.session_state.obstacles)):
            h = st.session_state.obs_heights[i] if i < len(st.session_state.obs_heights) else 50
            status = "需要绕行" if st.session_state.drone_height < h else "可直接飞过"
            st.info(f"障碍物 {i+1}：高度 {h} 米 → {status}")
    else:
        st.info("暂无障碍物数据")
    
    # 航线信息
    st.subheader("📍 航线信息")
    if st.session_state.point_a and st.session_state.point_b:
        st.write(f"起点A：{st.session_state.point_a}")
        st.write(f"终点B：{st.session_state.point_b}")
        _, route_status = calculate_best_route()
        st.write(f"航线策略：{route_status}")
    else:
        st.warning("请先在「航线规划」页面设置起点和终点！")