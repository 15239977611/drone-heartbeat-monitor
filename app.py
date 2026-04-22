import streamlit as st
import folium
from streamlit_folium import st_folium
from shapely.geometry import LineString, Polygon

# ==============================================
# 初始化页面（必须放在最顶部）
# ==============================================
st.set_page_config(page_title="无人机避障航线规划", layout="wide")

# ==============================================
# 初始化 session_state（保存数据不丢失）
# ==============================================
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []  # 存储所有障碍物 [points, height]

if "click_points" not in st.session_state:
    st.session_state.click_points = []  # 临时圈选障碍物点

if "a_point" not in st.session_state:
    st.session_state.a_point = [116.3975, 39.9075]  # 默认A点

if "b_point" not in st.session_state:
    st.session_state.b_point = [116.4025, 39.9075]  # 默认B点

# ==============================================
# 侧边栏：参数设置
# ==============================================
st.sidebar.title("🛠️ 参数设置")
drone_height = st.sidebar.number_input("无人机飞行高度（m）", value=38)
obstacle_height = st.sidebar.number_input("障碍物高度（m）", value=50)

# ==============================================
# 核心：绝对避障判断函数（你要的关键逻辑）
# ==============================================
def is_path_absolutely_safe(line_coords, obstacles, drone_h):
    flight_line = LineString(line_coords)

    # 遍历所有障碍物
    for obs in obstacles:
        obs_points, obs_h = obs
        if len(obs_points) < 3:
            continue

        # 构建障碍物多边形
        try:
            obs_poly = Polygon([(p["lng"], p["lat"]) for p in obs_points])
        except:
            continue

        # =======================
        # 你的铁律：只要障碍物更高 → 绝对不能有任何相交/重叠
        # =======================
        if obs_h > drone_h:
            if flight_line.intersects(obs_poly):
                return False, "❌ 航线与高障碍物重叠，禁止飞行"

    return True, "✅ 航线安全"

# ==============================================
# 地图主体
# ==============================================
st.markdown("## 🛸 无人机航线规划（绝对避障版）")

# 创建地图
m = folium.Map(location=[39.9075, 116.4000], zoom_start=18, control_scale=True)

# 绘制A点
folium.CircleMarker(
    location=st.session_state.a_point[::-1],
    radius=8, color="blue", fill=True, fill_color="blue", popup="A点"
).add_to(m)

# 绘制B点
folium.CircleMarker(
    location=st.session_state.b_point[::-1],
    radius=8, color="red", fill=True, fill_color="red", popup="B点"
).add_to(m)

# 航线坐标
path_coords = [
    (st.session_state.a_point[0], st.session_state.a_point[1]),
    (st.session_state.b_point[0], st.session_state.b_point[1])
]

# ==============================================
# ✅ 执行：绝对安全判断
# ==============================================
safe, msg = is_path_absolutely_safe(path_coords, st.session_state.obstacles, drone_height)

# 只有安全才画航线
if safe:
    folium.PolyLine(
        locations=[p[::-1] for p in path_coords],
        color="blue", weight=5, opacity=0.8
    ).add_to(m)

# ==============================================
# 绘制所有障碍物
# ==============================================
for idx, obs in enumerate(st.session_state.obstacles):
    points, h = obs
    if len(points) < 3:
        continue

    color = "red" if h > drone_height else "orange"
    folium.Polygon(
        locations=[[p["lat"], p["lng"]] for p in points],
        color=color, fill=True, fill_opacity=0.4, popup=f"障碍物{idx+1} 高度：{h}m"
    ).add_to(m)

# ==============================================
# 渲染地图
# ==============================================
output = st_folium(m, width="100%", height=600)

# ==============================================
# 障碍物操作按钮
# ==============================================
st.sidebar.markdown("---")
st.sidebar.subheader("障碍物操作")

if st.sidebar.button("📌 添加当前点到障碍物"):
    if output and output.get("last_clicked"):
        lat = output["last_clicked"]["lat"]
        lng = output["last_clicked"]["lng"]
        st.session_state.click_points.append({"lat": lat, "lng": lng})
        st.sidebar.success(f"已添加点：({lat:.5f}, {lng:.5f})")

if st.sidebar.button("✅ 确认生成障碍物"):
    if len(st.session_state.click_points) >= 3:
        st.session_state.obstacles.append([
            st.session_state.click_points,
            obstacle_height
        ])
        st.session_state.click_points = []
        st.sidebar.success("✅ 障碍物添加成功！")
    else:
        st.sidebar.error("至少需要3个点才能生成障碍物")

if st.sidebar.button("🗑️ 清空所有障碍物"):
    st.session_state.obstacles = []
    st.session_state.click_points = []
    st.sidebar.success("已清空所有障碍物")

# ==============================================
# 显示结果
# ==============================================
st.markdown("---")
if safe:
    st.success(msg)
else:
    st.error(msg)
    st.warning("⚠️ 障碍物高度 > 无人机高度 → 航线禁止重叠！")