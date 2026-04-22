import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString
import math
from datetime import datetime

# ================== 初始化 ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
# 合并存储：自定义障碍物 + 原生地理实体（房屋/树等）
if "obstacles_all" not in st.session_state:
    st.session_state.obstacles_all = []
if "obstacles_type" not in st.session_state:
    st.session_state.obstacles_type = []  # 类型：自定义/房屋/高楼/树木
if "obstacles_height" not in st.session_state:
    st.session_state.obstacles_height = []
if "drawing_mode" not in st.session_state:
    st.session_state.drawing_mode = None  # 绘制类型：None/自定义/房屋/高楼/树木
if "current_points" not in st.session_state:
    st.session_state.current_points = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 50

# ================== 预设常见实体高度（贴近真实） ==================
DEFAULT_HEIGHTS = {
    "自定义障碍物": 50,
    "普通房屋": 20,       # 3-6层住宅
    "高层楼房": 80,       # 20+层高楼
    "大树/电线杆": 10     # 树木/小型设施
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
st.set_page_config(page_title="真实场景无人机避障系统", layout="wide")

# ================== 核心：全场景避障算法 ==================
def calculate_smart_route():
    """
    核心逻辑：
    1. 识别所有实体（自定义/房屋/高楼/树木）
    2. 对比高度：实体高度 > 无人机高度 → 绕行
    3. 生成最短、平滑的绕行路线
    """
    A = st.session_state.point_a
    B = st.session_state.point_b
    
    if not A or not B:
        return [], "未设置起点/终点"
    
    direct_line = LineString([A, B])
    drone_h = st.session_state.drone_height
    need_avoid = False
    avoid_points = []

    # 遍历所有实体（自定义+原生地理实体）
    for i, obs_coords in enumerate(st.session_state.obstacles_all):
        if len(obs_coords) < 3:
            continue
        
        # 创建实体多边形
        obs_poly = Polygon(obs_coords)
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        
        # 判断1：无人机高度足够 → 直接飞过
        if drone_h > obs_h:
            continue
        
        # 判断2：航线穿过实体 → 必须绕行
        if obs_poly.intersects(direct_line):
            need_avoid = True
            
            # 计算最优绕行点（最短路径）
            centroid = obs_poly.centroid
            ab_vector = (B[0]-A[0], B[1]-A[1])
            perp_vector = (-ab_vector[1], ab_vector[0])  # 垂直偏移方向
            scale = 0.0003 / math.hypot(*perp_vector) if math.hypot(*perp_vector) > 0 else 0.0003
            offset_x = perp_vector[0] * scale
            offset_y = perp_vector[1] * scale
            
            avoid_point = (centroid.x + offset_x, centroid.y + offset_y)
            avoid_points.append(avoid_point)
    
    # 生成最终路线
    if need_avoid and avoid_points:
        final_route = [A] + avoid_points + [B]
        status = f"需要绕行！无人机高度({drone_h}m) < 地理实体/障碍物高度"
    else:
        final_route = [A, B]
        status = f"直线飞行！无人机高度({drone_h}m) ≥ 所有地理实体/障碍物高度"
    
    return final_route, status

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("真实场景无人机避障系统")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    # 无人机高度设置
    st.markdown("---")
    st.subheader("🛸 无人机飞行高度")
    st.session_state.drone_height = st.slider(
        "设置飞行高度（米）",
        min_value=0, max_value=200, value=50, step=1,
        key="drone_height_slider"
    )

    # 地理实体绘制（核心新增）
    st.markdown("---")
    st.subheader("🌍 地理实体/障碍物圈选")
    st.info("圈选地图上的房屋/高楼/树木/自定义障碍物")
    
    # 选择绘制类型
    draw_type = st.selectbox(
        "选择要圈选的实体类型",
        ["无", "自定义障碍物", "普通房屋", "高层楼房", "大树/电线杆"],
        key="draw_type_select"
    )
    
    # 绘制控制按钮
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选") and draw_type != "无":
            st.session_state.drawing_mode = draw_type
            st.session_state.current_points = []
            st.success(f"开始圈选「{draw_type}」...")
    with col2:
        if st.button("✅ 完成圈选") and st.session_state.drawing_mode:
            if len(st.session_state.current_points) >= 3:
                # 添加实体数据
                st.session_state.obstacles_all.append(st.session_state.current_points)
                st.session_state.obstacles_type.append(st.session_state.drawing_mode)
                # 用预设高度，也可后续修改
                st.session_state.obstacles_height.append(DEFAULT_HEIGHTS[st.session_state.drawing_mode])
                save_all()
                st.success(f"「{st.session_state.drawing_mode}」添加成功！")
            else:
                st.warning("至少需要3个点才能形成多边形！")
            st.session_state.drawing_mode = None
            st.session_state.current_points = []

    # 清除所有实体
    if st.button("🗑️ 清空所有实体/障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_type = []
        st.session_state.obstacles_height = []
        save_all()
        st.success("所有实体/障碍物已清空！")

    # 实体高度修改（每个独立滑块）
    st.markdown("---")
    st.subheader("📏 实体高度自定义")
    if len(st.session_state.obstacles_all) > 0:
        for i in range(len(st.session_state.obstacles_all)):
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            current_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else DEFAULT_HEIGHTS[obs_type]
            
            st.write(f"📌 {obs_type} {i+1}")
            new_h = st.slider(
                f"高度（米）",
                min_value=0, max_value=200, value=current_h, step=1,
                key=f"obs_{i}_height",
                label_visibility="collapsed"
            )
            # 实时更新并保存
            if i < len(st.session_state.obstacles_height):
                st.session_state.obstacles_height[i] = new_h
                save_all()
    else:
        st.info("暂无实体，先圈选地图上的房屋/高楼等")

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
    st.title("🗺️ 真实场景无人机智能避障系统")
    
    # 计算最佳路线
    route, route_status = calculate_smart_route()
    
    # 显示航线状态
    st.info(f"📊 航线策略：{route_status}")

    # 初始化卫星地图（显示真实地理实体）
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

    # 绘制所有地理实体/障碍物（不同类型不同颜色）
    TYPE_COLORS = {
        "自定义障碍物": "darkred",
        "普通房屋": "orange",
        "高层楼房": "darkblue",
        "大树/电线杆": "darkgreen"
    }
    for i, obs in enumerate(st.session_state.obstacles_all):
        if len(obs) > 2:
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else DEFAULT_HEIGHTS[obs_type]
            color = TYPE_COLORS.get(obs_type, "darkred")
            
            folium.Polygon(
                locations=obs,
                color=color, fill=True, fill_color=color, fill_opacity=0.4,
                weight=4, 
                popup=f"{obs_type} | 高度：{obs_h}米",
                tooltip=f"{obs_type}（{obs_h}米）"
            ).add_to(m)

    # 绘制正在圈选的实体
    if st.session_state.drawing_mode and len(st.session_state.current_points) > 0:
        draw_type = st.session_state.drawing_mode
        color = TYPE_COLORS.get(draw_type, "orange")
        
        folium.PolyLine(
            locations=st.session_state.current_points,
            color=color, weight=4, dash_array='5,5',
            popup=f"正在绘制：{draw_type}"
        ).add_to(m)
        # 绘制每个顶点
        for idx, p in enumerate(st.session_state.current_points):
            folium.CircleMarker(
                location=p, radius=5, color=color, fill=True,
                popup=f"顶点 {idx+1}"
            ).add_to(m)

    # 绘制最终避障航线（蓝色高亮）
    if len(route) >= 2:
        folium.PolyLine(
            locations=route,
            color='blue', weight=6, opacity=0.9,
            popup=route_status,
            tooltip="最佳避障航线"
        ).add_to(m)

    # 地图交互
    map_out = st_folium(
        m, key="drone_map", height=750,
        use_container_width=True, returned_objects=["last_clicked"]
    )

    # 处理地图点击（设A/B点 或 绘制实体）
    if map_out and map_out.get("last_clicked"):
        lat = round(map_out["last_clicked"]["lat"], 6)
        lng = round(map_out["last_clicked"]["lng"], 6)
        
        if st.session_state.drawing_mode:
            # 绘制实体：添加顶点
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
    st.success(f"已圈选地理实体数量：{len(st.session_state.obstacles_all)} 个")
    
    # 实体详情列表
    if len(st.session_state.obstacles_all) > 0:
        st.subheader("🌍 地理实体/障碍物详情")
        for i in range(len(st.session_state.obstacles_all)):
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else DEFAULT_HEIGHTS[obs_type]
            status = "⚠️ 需要绕行" if st.session_state.drone_height < obs_h else "✅ 可直接飞过"
            st.info(f"{obs_type} {i+1}：高度 {obs_h} 米 → {status}")
    else:
        st.info("暂无地理实体/障碍物数据，请先圈选！")
    
    # 航线信息
    st.subheader("📍 航线信息")
    if st.session_state.point_a and st.session_state.point_b:
        st.write(f"起点A：{st.session_state.point_a}")
        st.write(f"终点B：{st.session_state.point_b}")
        _, route_status = calculate_smart_route()
        st.write(f"当前航线策略：{route_status}")
    else:
        st.warning("请先在「航线规划」页面设置起点和终点！")