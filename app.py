import streamlit as st
import folium
from streamlit_folium import st_folium
from shapely.geometry import LineString, Polygon

# 页面配置
st.set_page_config(layout="wide")

# 初始化状态（保留你原来的逻辑）
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "temp_points" not in st.session_state:
    st.session_state.temp_points = []
if "A" not in st.session_state:
    st.session_state.A = [116.3975, 39.9075]
if "B" not in st.session_state:
    st.session_state.B = [116.4025, 39.9075]

# ======================
# 侧边栏（你原来的界面）
# ======================
st.sidebar.title("无人机航线规划")
mode = st.sidebar.radio("模式", ["航线规划", "飞行监控"])
drone_height = st.sidebar.number_input("无人机飞行高度", value=38)
obstacle_height = st.sidebar.number_input("障碍物高度", value=50)

# ======================
# 核心：你要的绝对不重叠判断（唯一新增的代码）
# ======================
def check_safe(A, B, obstacles, drone_h):
    line = LineString([(A[0], A[1]), (B[0], B[1])])
    for obs in obstacles:
        pts, h = obs
        if len(pts) < 3:
            continue
        try:
            poly = Polygon([(p[0], p[1]) for p in pts])
        except:
            continue
        # 铁律：障碍物更高 → 任何相交都不行
        if h > drone_h and line.intersects(poly):
            return False
    return True

# ======================
# 地图（完全保留你原来的样式）
# ======================
m = folium.Map(location=[39.9075, 116.40], zoom_start=17)

# 画A点 B点
folium.CircleMarker(location=st.session_state.A[::-1], radius=8, color="blue", fill=True, popup="A").add_to(m)
folium.CircleMarker(location=st.session_state.B[::-1], radius=8, color="red", fill=True, popup="B").add_to(m)

# 障碍物绘制（保留你原来功能）
for pts, h in st.session_state.obstacles:
    if len(pts) >=3:
        color = "red" if h > drone_height else "orange"
        folium.Polygon(locations=[p[::-1] for p in pts], color=color, fill=True, fill_opacity=0.5).add_to(m)

# ======================
# ✅ 关键：只有安全才画航线（绝对不重叠）
# ======================
safe = check_safe(st.session_state.A, st.session_state.B, st.session_state.obstacles, drone_height)
if safe:
    folium.PolyLine(
        locations=[st.session_state.A[::-1], st.session_state.B[::-1]],
        color="blue", weight=5
    ).add_to(m)

# ======================
# 渲染地图（保证一定显示）
# ======================
map_output = st_folium(m, key="map", width="100%", height=600)

# ======================
# 障碍物操作（你原来的功能）
# ======================
st.sidebar.subheader("障碍物操作")
if st.sidebar.button("点击地图添加点位"):
    if map_output and map_output.get("last_clicked"):
        lat = map_output["last_clicked"]["lat"]
        lng = map_output["last_clicked"]["lng"]
        st.session_state.temp_points.append([lng, lat])
        st.sidebar.success(f"已加点：{len(st.session_state.temp_points)}个")

if st.sidebar.button("确认生成障碍物"):
    if len(st.session_state.temp_points) >=3:
        st.session_state.obstacles.append([st.session_state.temp_points, obstacle_height])
        st.session_state.temp_points = []
        st.sidebar.success("障碍物生成成功")
    else:
        st.sidebar.error("至少3个点")

if st.sidebar.button("清空障碍物"):
    st.session_state.obstacles = []
    st.session_state.temp_points = []
    st.sidebar.success("已清空")

# ======================
# 结果提示
# ======================
st.markdown("---")
if safe:
    st.success("✅ 航线安全，无重叠")
else:
    st.error("❌ 障碍物高度＞无人机高度 → 航线禁止重叠，已隐藏航线")