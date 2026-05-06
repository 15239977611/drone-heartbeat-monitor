import streamlit as st
import json
import math
import time
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points

# ================== 初始化 ==================
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "route" not in st.session_state:
    st.session_state.route = []
if "flight_status" not in st.session_state:
    st.session_state.flight_status = "✅ 等待起飞"
if "flight_progress" not in st.session_state:
    st.session_state.flight_progress = 0
if "map_data" not in st.session_state:
    st.session_state.map_data = {"a": None, "b": None, "obs": []}

# ================== 坐标转换 ==================
def latlon_to_xy(lat, lon, ref_lat=32.2330, ref_lon=118.7490):
    R = 6378137
    x = (lon - ref_lon) * math.pi / 180 * R * math.cos(ref_lat * math.pi / 180)
    y = (lat - ref_lat) * math.pi / 180 * R
    return round(x, 2), round(y, 2)

def xy_to_latlon(x, y, ref_lat=32.2330, ref_lon=118.7490):
    R = 6378137
    d_lat = (y / R) * 180 / math.pi
    d_lon = (x / (R * math.cos(ref_lat * math.pi / 180))) * 180 / math.pi
    return round(ref_lat + d_lat, 6), round(ref_lon + d_lon, 6)

# ================== 绕行算法（含安全半径）==================
def avoid_obstacles(A, B, obstacles, drone_alt, obs_alt, safety_radius):
    path = [A]
    line = LineString([A, B])

    if drone_alt >= obs_alt:
        return [A, B]

    for obs in obstacles:
        if len(obs) < 3:
            continue
        poly = Polygon(obs)
        buffered = poly.buffer(safety_radius)
        if line.intersects(buffered):
            p1, p2 = nearest_points(line, buffered.boundary)
            px1, py1 = p1.x, p1.y
            px2, py2 = p2.x, p2.y
            path.append((py1, px1))
            path.append((py2, px2))

    path.append(B)
    return path

# ================== 界面 ==================
st.set_page_config(layout="wide")
st.title("✅ 无人机完整巡航系统（可绕行+坐标转换+监控+安全半径）")

col1, col2 = st.columns([3, 1])

with col2:
    st.subheader("🛠 参数设置")
    drone_alt = st.slider("无人机高度 (m)", 0, 50, 10)
    obs_alt = st.slider("障碍物高度 (m)", 0, 50, 25)
    safety_radius = st.slider("安全半径 (m)", 0, 20, 5)

    st.divider()
    st.subheader("📍 清除点位")
    if st.button("🟢 清除 A 点"):
        st.session_state.point_a = None
        st.session_state.map_data["a"] = None
    if st.button("🔴 清除 B 点"):
        st.session_state.point_b = None
        st.session_state.map_data["b"] = None
    if st.button("🧹 清除所有点位 & 障碍物"):
        st.session_state.point_a = None
        st.session_state.point_b = None
        st.session_state.obstacles = []
        st.session_state.route = []

    st.divider()
    st.subheader("📍 坐标转换")
    if st.session_state.point_a:
        x, y = latlon_to_xy(*st.session_state.point_a)
        st.write(f"A 局部坐标：X={x} Y={y}")
    if st.session_state.point_b:
        x, y = latlon_to_xy(*st.session_state.point_b)
        st.write(f"B 局部坐标：X={x} Y={y}")

    st.divider()
    st.subheader("✈️ 飞行监控")
    st.write(f"状态：{st.session_state.flight_status}")
    st.progress(st.session_state.flight_progress)

    if st.button("📐 生成航线 & 绕行"):
        A = st.session_state.point_a
        B = st.session_state.point_b
        if A and B:
            route = avoid_obstacles(
                A, B,
                st.session_state.obstacles,
                drone_alt, obs_alt,
                safety_radius
            )
            st.session_state.route = route
            st.success("✅ 航线已生成！")
        else:
            st.warning("请先设置 A 和 B")

    if st.button("🚀 开始飞行"):
        if not st.session_state.route:
            st.warning("先生成航线！")
        else:
            st.session_state.flight_status = "🛫 飞行中"
            st.session_state.flight_progress = 0
            for i in range(101):
                st.session_state.flight_progress = i
                time.sleep(0.02)
            st.session_state.flight_status = "✅ 已到达目的地"

# ================== 地图 ==================
with col1:
    init_data = {
        "a": st.session_state.point_a,
        "b": st.session_state.point_b,
        "obs": st.session_state.obstacles,
        "route": st.session_state.route
    }

    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8"/>
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>
            #map { height: 750px; width: 100%; }
            .panel { position: absolute; top:10; left:10; z-index:1000; background:white; padding:8px; border-radius:4px; }
        </style>
    </head>
    <body>
        <div id="map"></div>
        <script>
            var map = L.map('map').setView([32.2330, 118.7490], 18);
            L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}').addTo(map);

            var initData = __INIT_DATA__;
            var pointA = initData.a;
            var pointB = initData.b;
            var obstacles = initData.obs;
            var route = initData.route;

            var mA, mB;
            var obsLayers = [];
            var routeLine;

            function drawAll() {
                if (pointA) {
                    if (mA) map.removeLayer(mA);
                    mA = L.marker(pointA).addTo(map).bindPopup("A 起点");
                }
                if (pointB) {
                    if (mB) map.removeLayer(mB);
                    mB = L.marker(pointB).addTo(map).bindPopup("B 终点");
                }
                if (pointA && pointB) {
                    L.polyline([pointA, pointB], {color:"green", weight:3}).addTo(map);
                }
                obstacles.forEach(o => {
                    L.polygon(o, {color:"orange"}).addTo(map);
                });
                if (route && route.length > 1) {
                    if (routeLine) map.removeLayer(routeLine);
                    routeLine = L.polyline(route, {color:"blue", weight:4}).addTo(map);
                }
            }

            map.on('click', function(e) {
                var lat = e.latlng.lat;
                var lng = e.latlng.lng;
                var data = { a:pointA, b:pointB, obs:obstacles };

                if (!pointA) {
                    pointA = [lat, lng];
                } else if (!pointB) {
                    pointB = [lat, lng];
                } else {
                    obstacles.push([lat, lng]);
                }
                drawAll();

                window.parent.postMessage({
                    type: "streamlit:setComponentValue",
                    value: { a:pointA, b:pointB, obs:obstacles }
                }, "*");
            });

            drawAll();
        </script>
    </body>
    </html>
    """

    html_code = html_template.replace("__INIT_DATA__", json.dumps(init_data))
    st.components.v1.html(html_code, height=750, scrolling=False)

# ================== 接收地图数据（零报错）==================
try:
    data = st.session_state.get("component_value")
    if data and isinstance(data, dict):
        if data.get("a"):
            st.session_state.point_a = tuple(data["a"])
            st.session_state.map_data["a"] = data["a"]
        if data.get("b"):
            st.session_state.point_b = tuple(data["b"])
            st.session_state.map_data["b"] = data["b"]
        if data.get("obs") is not None:
            st.session_state.obstacles = data["obs"]
except:
    pass