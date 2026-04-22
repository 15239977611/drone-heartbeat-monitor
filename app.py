import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString
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

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("导航")
    page = st.radio("选择页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    # 无人机飞行高度（滑块）
    st.subheader("🛸 无人机飞行高度")
    st.session_state.drone_height = st.slider(
        "飞行高度（米）",
        min_value=0, max_value=200, value=50, step=1,
        key="drone_h_slider"
    )

    st.markdown("---")
    # 障碍物绘制
    st.subheader("🚧 实体障碍物（禁区）")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选禁区"):
            st.session_state.drawing_obstacle = True
            st.session_state.current_points = []
    with col2:
        if st.button("✅ 完成圈选"):
            if len(st.session_state.current_points) >= 3:
                st.session_state.obstacles.append(st.session_state.current_points)
                # 默认高度50米，和无人机初始高度一致
                st.session_state.obs_heights.append(50)
                save_all()
            st.session_state.drawing_obstacle = False
            st.session_state.current_points = []

    # 清除所有障碍物
    if st.button("🗑️ 清除所有障碍物"):
        st.session_state.obstacles = []
        st.session_state.obs_heights = []
        save_all()

    # ========== 核心新增：每个障碍物独立滑块改高度 ==========
    st.markdown("---")
    st.subheader("📏 障碍物高度设置（可修改）")
    if len(st.session_state.obstacles) > 0:
        for i in range(len(st.session_state.obstacles)):
            # 每个障碍物一个独立滑块
            new_height = st.slider(
                f"障碍物 {i+1} 高度（米）",
                min_value=0, max_value=200,
                value=st.session_state.obs_heights[i] if i < len(st.session_state.obs_heights) else 50,
                step=1,
                key=f"obs_{i}_height"
            )
            # 实时更新高度
            if i < len(st.session_state.obs_heights):
                st.session_state.obs_heights[i] = new_height
                save_all()  # 自动保存修改后的高度
    else:
        st.info("暂无障碍物，先圈选禁区")

    st.markdown("---")
    # A/B点设置
    st.subheader("🟢 A 点（起点）")
    if st.session_state.point_a:
        st.success(f"纬度：{st.session_state.point_a[0]:.6f}")
        st.success(f"经度：{st.session_state.point_a[1]:.6f}")
        if st.button("🗑️ 清除 A 点"):
            st.session_state.point_a = None
    else:
        st.warning("未设置")

    st.markdown("---")
    st.subheader("🔴 B 点（终点）")
    if st.session_state.point_b:
        st.success(f"纬度：{st.session_state.point_b[0]:.6f}")
        st.success(f"经度：{st.session_state.point_b[1]:.6f}")
        if st.button("🗑️ 清除 B 点"):
            st.session_state.point_b = None
    else:
        st.warning("未设置")

# ================== 最佳绕行算法 ==================
def best_path():
    A = st.session_state.point_a
    B = st.session_state.point_b
    if not A or not B:
        return []

    waypoints = [A]
    path_line = LineString([A, B])

    # 遍历所有障碍物，判断是否需要绕行
    for obs in st.session_state.obstacles:
        try:
            poly = Polygon(obs)
            if poly.intersects(path_line):
                # 计算平滑绕行点（避开障碍物中心）
                centroid = poly.centroid
                # 偏移距离（保证绕开障碍物）
                offset = 0.00025
                # 计算绕行方向
                dx = centroid.x - A[0]
                dy = centroid.y - A[1]
                side_point = (
                    centroid.x + (-dy) * offset,
                    centroid.y + dx * offset
                )
                waypoints.append(side_point)
        except:
            continue

    waypoints.append(B)
    return waypoints

# ================== 地图 ==================
if page == "航线规划":
    st.title("🗺️ 无人机智能绕行系统")
    st.success(f"✅ 无人机高度：{st.session_state.drone_height}m | 障碍物高度可实时修改")

    # 卫星地图（修复归属，正常显示）
    m = folium.Map(
        location=[32.2330, 118.7490],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery"
    )

    # 绘制A/B点
    if st.session_state.point_a:
        folium.CircleMarker(
            location=st.session_state.point_a,
            radius=9, color='green', fill=True, fill_color='green'
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_a,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px;">A 起点</div>')
        ).add_to(m)
    
    if st.session_state.point_b:
        folium.CircleMarker(
            location=st.session_state.point_b,
            radius=9, color='red', fill=True, fill_color='red'
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_b,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px;">B 终点</div>')
        ).add_to(m)

    # 绘制障碍物（显示高度）
    for i, obs in enumerate(st.session_state.obstacles):
        if len(obs) > 2:
            h = st.session_state.obs_heights[i] if i < len(st.session_state.obs_heights) else 50
            folium.Polygon(
                locations=obs,
                color='red', fill=True, fill_color='red', fill_opacity=0.4,
                weight=3, popup=f"障碍物 {i+1} | 高度：{h}m"
            ).add_to(m)

    # 绘制最佳绕行路线
    path = best_path()
    if len(path) > 1:
        folium.PolyLine(
            locations=path, color='blue', weight=5, opacity=0.8,
            popup="最佳绕行航线"
        ).add_to(m)

    # 绘制中的障碍物
    if st.session_state.drawing_obstacle and len(st.session_state.current_points) > 0:
        folium.PolyLine(
            locations=st.session_state.current_points,
            color='orange', weight=4, opacity=0.8
        ).add_to(m)
        # 绘制每个顶点
        for p in st.session_state.current_points:
            folium.CircleMarker(location=p, radius=4, color='orange', fill=True).add_to(m)

    # 地图交互（点击设点/绘制）
    map_out = st_folium(
        m, key="map", height=700,
        use_container_width=True, returned_objects=["last_clicked"]
    )

    # 处理地图点击
    if map_out and map_out.get("last_clicked"):
        lat = round(map_out["last_clicked"]["lat"], 6)
        lng = round(map_out["last_clicked"]["lng"], 6)
        if st.session_state.drawing_obstacle:
            # 绘制障碍物时添加顶点
            st.session_state.current_points.append([lat, lng])
        else:
            # 优先设A点，再设B点
            if not st.session_state.point_a:
                st.session_state.point_a = (lat, lng)
            else:
                st.session_state.point_b = (lat, lng)

# ================== 飞行监控 ==================
else:
    st.title("📡 飞行监控")
    st.success(f"🛸 无人机当前飞行高度：{st.session_state.drone_height} m")
    st.success("✅ 智能绕行系统已启动 | 障碍物高度实时同步")
    
    # 显示所有障碍物高度
    if len(st.session_state.obstacles) > 0:
        st.subheader("🚧 障碍物列表")
        for i in range(len(st.session_state.obstacles)):
            h = st.session_state.obs_heights[i] if i < len(st.session_state.obs_heights) else 50
            st.info(f"障碍物 {i+1}：高度 {h} 米")
    else:
        st.info("暂无已设置的障碍物")