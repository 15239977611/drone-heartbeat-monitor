import streamlit as st
import streamlit.components.v1 as components
import json
import math
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
from shapely.affinity import scale

# ==================== 页面初始化 ====================
st.set_page_config(page_title="无人机避障航线规划", layout="wide")

# ==================== 状态管理 ====================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles_all" not in st.session_state:
    st.session_state.obstacles_all = []
if "obstacles_height" not in st.session_state:
    st.session_state.obstacles_height = []
if "drone_height" not in st.session_state:
    st.session_state.drone_height = 8
if "drone_safety_radius" not in st.session_state:
    st.session_state.drone_safety_radius = 15
if "avoid_direction" not in st.session_state:
    st.session_state.avoid_direction = "左"
if "route" not in st.session_state:
    st.session_state.route = []
if "map_key" not in st.session_state:
    st.session_state.map_key = 0

REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50,
    "普通房屋": 20,
    "高层楼房": 80,
    "大树/电线杆": 10,
    "操场/空地": 0,
    "桥梁/高架": 15,
    "塔楼/信号塔": 60
}

# ==================== 核心：避障航线计算（已修复） ====================
def calculate_route(A, B, obstacles_all, heights, drone_h, safety_r, direction):
    if not A or not B:
        return []

    path = [A]
    current_pos = A

    try:
        lat_base = A[0]
        lat_per_m = 1.0 / 111000.0
        lng_per_m = 1.0 / (111000.0 * math.cos(math.radians(lat_base)))
        offset_lat = safety_r * lat_per_m
        offset_lng = safety_r * lng_per_m
    except:
        offset_lat = 0.0001
        offset_lng = 0.0001

    for obs, h in zip(obstacles_all, heights):
        if len(obs) < 3:
            continue
        if h <= drone_h:
            continue

        try:
            fixed_obs = [(p[1], p[0]) for p in obs]
            poly = Polygon(fixed_obs)
            current_pt = (current_pos[1], current_pos[0])
            b_pt = (B[1], B[0])
            line = LineString([current_pt, b_pt])

            if not poly.intersects(line):
                continue

            centroid = poly.centroid
            cx, cy = centroid.x, centroid.y

            if direction == "左":
                wp_lng = cx - offset_lng * 2.5
                wp_lat = cy
            else:
                wp_lng = cx + offset_lng * 2.5
                wp_lat = cy

            way_point = (wp_lat, wp_lng)
            path.append(way_point)
            current_pos = way_point

        except Exception as e:
            continue

    path.append(B)
    return path

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🛠️ 参数设置")
    drone_height = st.number_input("无人机飞行高度（米）", 0, 200, value=8)
    drone_safety_radius = st.number_input("安全半径（米）", 1, 50, value=15)
    avoid_direction = st.selectbox("绕飞方向", ["左", "右"])

    obstacle_type = st.selectbox("障碍物类型", list(REAL_WORLD_HEIGHTS.keys()))
    obstacle_height = REAL_WORLD_HEIGHTS[obstacle_type]
    st.info(f"当前障碍物高度：{obstacle_height} 米")

    st.session_state.drone_height = drone_height
    st.session_state.drone_safety_radius = drone_safety_radius
    st.session_state.avoid_direction = avoid_direction

    if st.button("✅ 确认设置并计算航线"):
        A = st.session_state.point_a
        B = st.session_state.point_b
        if not A or not B:
            st.warning("请先设置起点A和终点B！")
        else:
            route = calculate_route(
                A, B,
                st.session_state.obstacles_all,
                st.session_state.obstacles_height,
                drone_height, drone_safety_radius, avoid_direction
            )
            st.session_state.route = route
            st.success(f"航线已生成！共 {len(route)} 个航点")
            st.session_state.map_key += 1

    if st.button("🗑️ 清空所有障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_height = []
        st.session_state.route = []
        st.success("已清空障碍物")
        st.session_state.map_key += 1

# ==================== 地图 HTML ====================
map_html = """
<div id="map" style="width:100%;height:750px;"></div>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
    // 初始化地图
    const map = L.map('map').setView([30.2870, 120.1531], 18);
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: '© Esri'
    }).addTo(map);

    // 状态
    let mode = "";
    let pointA = null;
    let pointB = null;
    let markerA = null;
    let markerB = null;
    let currentPolygon = [];
    let polygonLayer = null;
    let obstacleLayers = [];
    let routeLine = null;
    let flyingMarker = null;
    let flightPath = [];
    let flightIndex = 0;
    let flightInterval = null;

    // 按钮控制
    window.setMode = function(m) {
        mode = m;
        if (m === "draw") startDraw();
        if (m === "stopDraw") stopDraw();
    }

    window.clearAll = function() {
        obstacleLayers.forEach(l => l.remove());
        obstacleLayers = [];
        currentPolygon = [];
        sendData();
    }

    window.setA = function() { setMode("A"); }
    window.setB = function() { setMode("B"); }

    // 点击地图
    map.on("click", function(e) {
        const lat = e.latlng.lat;
        const lng = e.latlng.lng;

        if (mode === "A") {
            pointA = [lat, lng];
            if (markerA) markerA.remove();
            markerA = L.marker([lat, lng], {icon: L.icon({
                iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-green.png',
                iconSize: [25,41]
            })}).addTo(map).bindTooltip("起点A").openTooltip();
            mode = "";
            sendData();
        } else if (mode === "B") {
            pointB = [lat, lng];
            if (markerB) markerB.remove();
            markerB = L.marker([lat, lng], {icon: L.icon({
                iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
                iconSize: [25,41]
            })}).addTo(map).bindTooltip("终点B").openTooltip();
            mode = "";
            sendData();
        } else if (mode === "draw") {
            currentPolygon.push([lat, lng]);
            if (polygonLayer) polygonLayer.remove();
            polygonLayer = L.polygon(currentPolygon, {color: "orange", fillOpacity:0.4}).addTo(map);
        }
    });

    function startDraw() { currentPolygon = []; }
    function stopDraw() {
        if (currentPolygon.length >= 3) {
            const p = L.polygon(currentPolygon, {color: "orange", fillOpacity:0.4}).addTo(map);
            obstacleLayers.push(p);
        }
        currentPolygon = [];
        polygonLayer = null;
        mode = "";
        sendData();
    }

    // 绘制航线
    window.drawRoute = function(route) {
        if (routeLine) routeLine.remove();
        if (!route || route.length < 2) return;
        routeLine = L.polyline(route, {color: "#0066ff", weight:5, opacity:0.8}).addTo(map);
        flightPath = route;
    }

    // 飞行模拟
    window.startFly = function() {
        if (flightPath.length < 2) return;
        if (flyingMarker) flyingMarker.remove();
        flyingMarker = L.marker(flightPath[0], {icon: L.icon({
            iconUrl: 'https://cdn-icons-png.flaticon.com/512/733/733519.png',
            iconSize: [32,32]
        })}).addTo(map);
        flightIndex = 0;
        clearInterval(flightInterval);
        flightInterval = setInterval(flyStep, 40);
    }

    function flyStep() {
        if (flightIndex >= flightPath.length - 1) {
            clearInterval(flightInterval);
            return;
        }
        flightIndex += 1;
        flyingMarker.setLatLng(flightPath[flightIndex]);
    }

    window.stopFly = function() {
        clearInterval(flightInterval);
    }

    // 发送数据到Streamlit
    function sendData() {
        const obs = obstacleLayers.map(l => l.getLatLngs()[0].map(p => [p.lat, p.lng]));
        window.parent.postMessage({
            type: "streamlit:setComponentValue",
            value: {a: pointA, b: pointB, obstacles: obs}
        }, "*");
    }
</script>

<!-- 操作按钮 -->
<div style="padding:10px;background:#f5f5f5;">
    <button onclick="setA()" style="padding:8px 12px;margin:2px;">📍 设置起点A</button>
    <button onclick="setB()" style="padding:8px 12px;margin:2px;">📍 设置终点B</button>
    <button onclick="setMode('draw')" style="padding:8px 12px;margin:2px;background:orange;color:white;">✏️ 绘制障碍物</button>
    <button onclick="setMode('stopDraw')" style="padding:8px 12px;margin:2px;background:green;color:white;">✅ 完成障碍物</button>
    <button onclick="clearAll()" style="padding:8px 12px;margin:2px;background:red;color:white;">🗑️ 清空障碍物</button>
    <button onclick="startFly()" style="padding:8px 12px;margin:2px;background:blue;color:white;">✈️ 开始飞行</button>
    <button onclick="stopFly()" style="padding:8px 12px;margin:2px;">⏹️ 停止飞行</button>
</div>
"""

# ==================== 渲染地图 ====================
st.title("✈️ 无人机自主避障航线规划系统")
components.html(map_html, height=850, key=f"map_{st.session_state.map_key}")

# ==================== 接收数据 ====================
returned_data = st.session_state.get(f"component_map_{st.session_state.map_key}")
if returned_data is not None and isinstance(returned_data, dict):
    a = returned_data.get("a")
    b = returned_data.get("b")
    obs = returned_data.get("obstacles", [])

    if a: st.session_state.point_a = a
    if b: st.session_state.point_b = b
    if obs:
        st.session_state.obstacles_all = obs
        st.session_state.obstacles_height = [obstacle_height] * len(obs)

# ==================== 实时绘制航线 ====================
if st.session_state.route:
    components.html(f"""
    <script>
        window.drawRoute({json.dumps(st.session_state.route)});
    </script>
    """, height=0, key="draw_route")

st.markdown("---")
st.caption("✅ 操作步骤：设置A/B点 → 绘制障碍物 → 侧边栏计算航线 → 开始飞行")