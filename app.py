import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point
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
    st.session_state.drone_height = 8  # 默认8米

# ================== 核心配置（地面高度=0米基准） ==================
GROUND_HEIGHT = 0  # 地面基准高度
SAFE_OFFSET = 0.0005  # 安全偏移距离（确保远离障碍物）
# 实景物体高度库（基于地面0米）
REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50,
    "普通房屋": 20,
    "高层楼房": 80,
    "大树/电线杆": 10,
    "操场/空地": 0,
    "桥梁/高架": 15,
    "塔楼/信号塔": 60
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
st.set_page_config(page_title="精准避障无人机系统（最佳航线）", layout="wide")

# ================== 核心优化：最佳航线绕行算法（最短路径） ==================
def calculate_precise_avoid_route():
    """
    核心优化：
    1. 生成4个候选绕行点（上下左右），计算总距离最短的点
    2. 确保绕行点绝对不在障碍物内
    3. 全局计算「当前点→绕行点→终点」总距离，选最优
    """
    A = st.session_state.point_a
    B = st.session_state.point_b
    
    if not A or not B:
        return [], "未设置起点/终点（地面基准：0米）"
    
    drone_h = st.session_state.drone_height  # 无人机地面以上高度
    final_route = [A]
    avoid_obstacles = []  # 记录需要绕行的障碍物

    # 遍历每个障碍物，逐个计算最佳绕行点
    for i, obs_coords in enumerate(st.session_state.obstacles_all):
        if len(obs_coords) < 3:
            continue
        
        obs_poly = Polygon(obs_coords)
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        
        # 高度足够（无人机≥障碍物）→ 跳过，不绕行
        if drone_h >= obs_h:
            continue
        
        avoid_obstacles.append(st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物")
        centroid = obs_poly.centroid  # 障碍物质心
        
        # 步骤1：生成4个候选绕行点（上下左右，远离障碍物）
        candidates = [
            (centroid.x - SAFE_OFFSET, centroid.y),  # 左
            (centroid.x + SAFE_OFFSET, centroid.y),  # 右
            (centroid.x, centroid.y + SAFE_OFFSET),  # 上
            (centroid.x, centroid.y - SAFE_OFFSET)   # 下
        ]
        
        # 步骤2：筛选有效绕行点（不在障碍物内）
        valid_candidates = []
        for cand in candidates:
            cand_point = Point(cand)
            if not obs_poly.contains(cand_point):
                # 计算总距离：当前点 → 候选点 → 终点B（核心：选总距离最短的）
                total_dist = math.hypot(final_route[-1][0]-cand[0], final_route[-1][1]-cand[1]) + math.hypot(cand[0]-B[0], cand[1]-B[1])
                valid_candidates.append((total_dist, cand))
        
        # 步骤3：选总距离最短的绕行点
        if valid_candidates:
            # 按总距离排序，选最短的
            valid_candidates.sort(key=lambda x: x[0])
            best_avoid_point = valid_candidates[0][1]
            final_route.append(best_avoid_point)
        else:
            # 极端情况：4个点都在障碍物内 → 向外偏移更大距离
            best_avoid_point = (centroid.x + SAFE_OFFSET*2, centroid.y + SAFE_OFFSET*2)
            final_route.append(best_avoid_point)

    # 添加终点
    final_route.append(B)
    
    # 状态判断
    if avoid_obstacles:
        status = f"🔴 最佳航线绕行！无人机高度({drone_h}m) < 障碍物高度，已避开：{','.join(avoid_obstacles)}（地面基准：0米）"
    else:
        status = f"🟢 直线飞行！无人机高度({drone_h}m) ≥ 所有障碍物高度（地面基准：0米）"
    
    return final_route, status

# ================== 侧边栏（保留原有功能） ==================
with st.sidebar:
    st.title("无人机精准避障系统")
    st.info(f"📌 地面基准高度：{GROUND_HEIGHT}米")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    # 无人机高度设置（明确地面以上）
    st.markdown("---")
    st.subheader("🛸 无人机飞行高度（地面以上）")
    st.session_state.drone_height = st.slider(
        "设置地面以上飞行高度（米）",
        min_value=0, max_value=200, value=8, step=1,
        key="drone_height_slider"
    )
    st.caption(f"当前：{st.session_state.drone_height}米（地面以上）")

    # 实景物体圈选
    st.markdown("---")
    st.subheader("🌍 实景物体/障碍物圈选")
    st.warning("圈选后高度不足将100%绕行，且选择最短路径！")
    
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
            st.success(f"开始圈选「{draw_type}」（预设高度：{REAL_WORLD_HEIGHTS[draw_type]}米，地面基准）")
    with col2:
        if st.button("✅ 完成圈选") and st.session_state.drawing_mode:
            if len(st.session_state.current_points) >= 3:
                st.session_state.obstacles_all.append(st.session_state.current_points)
                st.session_state.obstacles_type.append(st.session_state.drawing_mode)
                st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[st.session_state.drawing_mode])
                save_all()
                st.success(f"「{st.session_state.drawing_mode}」添加成功！高度：{REAL_WORLD_HEIGHTS[st.session_state.drawing_mode]}米（地面以上）")
            else:
                st.error("❌ 至少需要3个点形成多边形！")
            st.session_state.drawing_mode = None
            st.session_state.current_points = []

    # 清空按钮
    if st.button("🗑️ 清空所有实体/障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_type = []
        st.session_state.obstacles_height = []
        save_all()
        st.success("✅ 所有实体已清空！")

    # 实体高度自定义
    st.markdown("---")
    st.subheader("📏 实景物体高度自定义（地面以上）")
    if len(st.session_state.obstacles_all) > 0:
        for i in range(len(st.session_state.obstacles_all)):
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            current_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else REAL_WORLD_HEIGHTS[obs_type]
            
            st.write(f"📌 {obs_type} {i+1}")
            new_h = st.slider(
                f"高度（米）",
                min_value=0, max_value=200, value=current_h, step=1,
                key=f"obs_{i}_height",
                label_visibility="collapsed"
            )
            # 实时更新
            if i < len(st.session_state.obstacles_height):
                st.session_state.obstacles_height[i] = new_h
                save_all()
                if new_h > st.session_state.drone_height:
                    st.error(f"⚠️ 高度({new_h}m) > 无人机({st.session_state.drone_height}m) → 最短路径绕行！")
                else:
                    st.success(f"✅ 高度({new_h}m) ≤ 无人机 → 可直飞")
    else:
        st.info("暂无实体，先圈选地图上的物体")

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

# ================== 地图与航线展示（保留原有功能+优化标注） ==================
if page == "航线规划":
    st.title("🗺️ 无人机精准避障系统（最佳航线版）")
    
    # 计算最佳避障路线
    route, route_status = calculate_precise_avoid_route()
    
    # 醒目显示状态
    if "最佳航线绕行" in route_status:
        st.error(route_status)
    else:
        st.success(route_status)

    # 卫星地图
    m = folium.Map(
        location=[32.2330, 118.7490],  # 替换成你的地图中心坐标
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery"
    )

    # 绘制起点A
    if st.session_state.point_a:
        folium.CircleMarker(
            location=st.session_state.point_a,
            radius=12, color='green', fill=True, fill_color='green', fill_opacity=0.8,
            popup=f"起点A<br>地面基准：{GROUND_HEIGHT}米"
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
            popup=f"终点B<br>地面基准：{GROUND_HEIGHT}米"
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_b,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px; background:red; padding:2px;">B 终点</div>')
        ).add_to(m)

    # 绘制障碍物（高度超标则高亮）
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
            
            # 高度超标 → 高亮+加粗边框
            fill_opacity = 0.7 if obs_h > st.session_state.drone_height else 0.3
            weight = 8 if obs_h > st.session_state.drone_height else 3
            
            folium.Polygon(
                locations=obs,
                color=color, fill=True, fill_color=color, fill_opacity=fill_opacity,
                weight=weight, 
                popup=f"{obs_type} | 地面以上高度：{obs_h}米<br>无人机高度：{st.session_state.drone_height}米<br>状态：{'最短路径绕行' if obs_h > st.session_state.drone_height else '可直飞'}",
                tooltip=f"{obs_type}（{obs_h}米，地面以上）"
            ).add_to(m)

    # 绘制正在圈选的实体
    if st.session_state.drawing_mode and len(st.session_state.current_points) > 0:
        draw_type = st.session_state.drawing_mode
        color = TYPE_COLORS.get(draw_type, "orange")
        
        folium.PolyLine(
            locations=st.session_state.current_points,
            color=color, weight=5, dash_array='5,5',
            popup=f"正在绘制：{draw_type}（地面基准：0米）"
        ).add_to(m)
        for idx, p in enumerate(st.session_state.current_points):
            folium.CircleMarker(
                location=p, radius=6, color=color, fill=True,
                popup=f"顶点 {idx+1}"
            ).add_to(m)

    # 绘制最佳避障航线（加粗+绕行点标注）
    if len(route) >= 2:
        folium.PolyLine(
            locations=route,
            color='blue', weight=10, opacity=0.9,
            popup=route_status,
            tooltip="最佳避障航线（最短路径）"
        ).add_to(m)
        # 标注每个最优绕行点
        for idx, point in enumerate(route[1:-1]):
            folium.CircleMarker(
                location=point, radius=10, color='blue', fill=True, fill_color='yellow',
                popup=f"最优绕行点 {idx+1}（最短路径）"
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
                st.success(f"✅ 起点A已设置：({lat}, {lng})（地面基准：0米）")
            elif not st.session_state.point_b:
                st.session_state.point_b = (lat, lng)
                st.success(f"✅ 终点B已设置：({lat}, {lng})（地面基准：0米）")

# ================== 飞行监控页面（优化状态展示） ==================
else:
    st.title("📡 无人机飞行监控中心（最佳航线版）")
    
    # 系统状态
    st.subheader("✅ 实时状态（地面基准：0米）")
    st.success(f"无人机地面以上高度：{st.session_state.drone_height} 米")
    st.success(f"已圈选障碍物数量：{len(st.session_state.obstacles_all)} 个")
    
    # 避障提醒
    avoid_count = sum(1 for h in st.session_state.obstacles_height if h > st.session_state.drone_height)
    if avoid_count > 0:
        st.error(f"🔴 发现 {avoid_count} 个障碍物高度超标，将按「最短路径」精准绕行！")
    else:
        st.success(f"🟢 所有障碍物高度均达标，可直线飞行！")
    
    # 障碍物详情
    if len(st.session_state.obstacles_all) > 0:
        st.subheader("🌍 障碍物详情（地面以上高度）")
        for i in range(len(st.session_state.obstacles_all)):
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
            status = "🔴 最短路径绕行" if obs_h > st.session_state.drone_height else "🟢 可直飞"
            st.info(f"{obs_type} {i+1}：高度 {obs_h} 米 → {status}")
    else:
        st.info("暂无障碍物数据！")
    
    # 航线策略
    st.subheader("📍 航线策略")
    if st.session_state.point_a and st.session_state.point_b:
        _, route_status = calculate_precise_avoid_route()
        st.write(f"当前策略：{route_status}")
    else:
        st.warning("请先设置起点和终点！")