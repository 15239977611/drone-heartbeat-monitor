import streamlit as st
import json
from shapely.geometry import Polygon, LineString
from shapely.ops import nearest_points
from shapely.affinity import scale
import math

# ================== 初始化 ==================
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
if "map_data" not in st.session_state or not isinstance(st.session_state.map_data, dict):
    st.session_state.map_data = {"a": None, "b": None, "obstacles": []}

REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50, "普通房屋": 20, "高层楼房": 80,
    "大树/电线杆": 10, "操场/空地": 0, "桥梁/高架": 15, "塔楼/信号塔": 60
}

# ================== 绕飞算法（修复核心逻辑） ==================
def calculate_route(A, B, obstacles_all, heights, drone_h, safety_r, direction):
    OFFSET = safety_r / 111000.0
    route = [A]

    for i, coords in enumerate(obstacles_all):
        if len(coords) < 3:
            continue
        h = heights[i] if i < len(heights) else 0
        if h <= drone_h:
            continue
        
        try:
            poly_coords = [(lng, lat) for lat, lng in coords]
            poly = Polygon(poly_coords)
            scale_factor = 1 + (safety_r / 111000.0) / max(poly.bounds[2]-poly.bounds[0], poly.bounds[3]-poly.bounds[1], 1e-6)
            safe_poly = scale(poly, xfact=scale_factor, yfact=scale_factor, origin='centroid')
            line = LineString([(route[-1][1], route[-1][0]), (B[1], B[0])])
            
            if not line.intersects(safe_poly):
                continue
            
            p, _ = nearest_points(line, safe_poly.boundary)
            px, py = p.x, p.y
            cx, cy = poly.centroid.x, poly.centroid.y
            dx, dy = px - cx, py - cy
            dist = math.hypot(dx, dy) or 1
            dx, dy = dx/dist, dy/dist
            
            if direction == "左":
                wx, wy = -dy, dx
            else:
                wx, wy = dy, -dx
            
            offset_lng = px + wx * OFFSET
            offset_lat = py + wy * OFFSET
            route.append((offset_lat, offset_lng))
            
        except Exception as e:
            continue
    
    route.append(B)
    return route

# ================== 侧边栏 ==================
st.set_page_config(layout="wide")
st.title("✈️ 无人机避障飞行系统（零闪烁版）")

with st.sidebar:
    st.subheader("🛸 飞行参数")
    st.session_state.drone_height = st.slider("飞行高度（米）", 0, 200, st.session_state.drone_height)
    st.session_state.drone_safety_radius = st.slider("安全半径（米）", 1, 50, st.session_state.drone_safety_radius)
    st.session_state.avoid_direction = st.radio("绕飞方向", ["左", "右"],
                                                index=0 if st.session_state.avoid_direction=="左" else 1)
    st.info(f"当前绕飞：{st.session_state.avoid_direction}")

    st.subheader("🌍 障碍物类型")
    obs_type = st.selectbox("类型（用于高度）", list(REAL_WORLD_HEIGHTS.keys()))
    obs_height = REAL_WORLD_HEIGHTS[obs_type]
    st.write(f"默认高度：{obs_height} 米")

    if st.button("🗑️ 清空所有障碍物（地图端）"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_height = []
        st.session_state.map_data["obstacles"] = []
        st.rerun()

    st.markdown("---")

    if st.button("📐 确认设置并计算航线"):
        A = None
        B = None
        if st.session_state.map_data.get("a"):
            A = tuple(st.session_state.map_data["a"])
        if st.session_state.map_data.get("b"):
            B = tuple(st.session_state.map_data["b"])
        obs = st.session_state.map_data.get("obstacles", [])

        if A and B:
            st.session_state.point_a = A
            st.session_state.point_b = B
            st.session_state.obstacles_all = obs
            st.session_state.obstacles_height = [obs_height] * len(obs) if obs else []
            route = calculate_route(
                A, B,
                st.session_state.obstacles_all,
                st.session_state.obstacles_height,
                st.session_state.drone_height,
                st.session_state.drone_safety_radius,
                st.session_state.avoid_direction
            )
            st.session_state.route = route
            st.success("✅ 航线计算完成！")
        else:
            st.warning("请先在地图上设置起点 A 和终点 B")

# ================== 传给 HTML 的数据 ==================
init_data = {
    "a": st.session_state.map_data.get("a"),
    "b": st.session_state.map_data.get("b"),
    "obstacles": st.session_state.map_data.get("obstacles", []),
    "route": st.session_state.route,
    "drone_height": st.session_state.drone_height,
    "safety_radius": st.session_state.drone_safety_radius,
    "avoid_direction": st.session_state.avoid_direction
}
init_data_json = json.dumps(init_data)

# ================== HTML / JS 地图（完全保留原有） ==================
html_template = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        html, body { height: 100%; margin: 0; padding: 0; }
        #map { height: 100%; width: 100%; }
        .btn-container {
            position: absolute; top: 10px; left: 10px; z-index: 1000;
            display: flex; gap: 5px; flex-wrap: wrap;
        }
        .btn-container button {
            background: white; border: 2px solid #333; border-radius: 4px;
            padding: 6px 12px; cursor: pointer; font-size: 14px; font-weight: bold;
        }
        .btn-container button.active {
            background: #4CAF50; color: white; border-color: #4CAF50;
        }
    </style>
</head>
<body>
    <div id="map"></div>
    <div class="btn-container">
        <button id="btn-set-a">🟢 设置起点 A</button>
        <button id="btn-set-b">🔴 设置终点 B</button>
        <button id="btn-draw-obs">✏️ 绘制障碍物</button>
        <button id="btn-finish-obs">✅ 完成障碍物</button>
        <button id="btn-clear-obs">🗑️ 清空障碍物</button>
        <button id="btn-fly">▶️ 开始飞行</button>
        <button id="btn-stop">⏹️ 停止飞行</button>
    </div>
    <script>
        var map = L.map('map').setView([32.2330, 118.7490], 18);
        L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            attribution: 'ESRI'
        }).addTo(map);

        var pointA = null, pointB = null, tempObsPoints = [], obstacles = [];
        var markersA = [], markersB = [], drawMode = false, drawLayer = null;
        var droneMarker = null, routePath = null, flightTimer = null;
        var flightPath = [], currentIndex = 0;

        var initData = __INIT_DATA__;

        if (initData.a) { pointA = initData.a; addMarkerA(pointA[0], pointA[1]); }
        if (initData.b) { pointB = initData.b; addMarkerB(pointB[0], pointB[1]); }
        if (initData.obstacles && initData.obstacles.length) {
            obstacles = initData.obstacles;
            obstacles.forEach(function(pts) { L.polygon(pts, {color: 'orange', fillOpacity: 0.4}).addTo(map); });
        }
        if (initData.route && initData.route.length >= 2) {
            drawRoute(initData.route);
            flightPath = interpolatePath(initData.route);
        }

        map.on('click', function(e) {
            if (drawMode) {
                tempObsPoints.push([e.latlng.lat, e.latlng.lng]);
                if (drawLayer) map.removeLayer(drawLayer);
                drawLayer = L.polyline(tempObsPoints, {color: 'red', dashArray: '5,5'}).addTo(map);
            }
        });

        document.getElementById('btn-set-a').onclick = function() {
            if (drawMode) return alert('请先完成障碍物绘制');
            map.once('click', function(e) {
                pointA = [e.latlng.lat, e.latlng.lng];
                addMarkerA(pointA[0], pointA[1]);
                sendDataToPython();
            });
        };
        document.getElementById('btn-set-b').onclick = function() {
            if (drawMode) return alert('请先完成障碍物绘制');
            map.once('click', function(e) {
                pointB = [e.latlng.lat, e.latlng.lng];
                addMarkerB(pointB[0], pointB[1]);
                sendDataToPython();
            });
        };
        document.getElementById('btn-draw-obs').onclick = function() {
            drawMode = true;
            tempObsPoints = [];
            if (drawLayer) map.removeLayer(drawLayer);
            this.classList.add('active');
            alert('现在点击地图添加障碍物顶点，完成后按「完成障碍物」');
        };
        document.getElementById('btn-finish-obs').onclick = function() {
            if (!drawMode) return alert('请先点击「绘制障碍物」');
            if (tempObsPoints.length < 3) return alert('至少需要3个点');
            obstacles.push(tempObsPoints.slice());
            L.polygon(tempObsPoints, {color: 'orange', fillOpacity: 0.4}).addTo(map);
            tempObsPoints = [];
            drawMode = false;
            if (drawLayer) map.removeLayer(drawLayer);
            document.getElementById('btn-draw-obs').classList.remove('active');
            sendDataToPython();
        };
        document.getElementById('btn-clear-obs').onclick = function() {
            obstacles = [];
            map.eachLayer(function(layer) {
                if (layer instanceof L.Polygon && layer.options.color === 'orange') {
                    map.removeLayer(layer);
                }
            });
            sendDataToPython();
        };
        document.getElementById('btn-fly').onclick = function() {
            if (!flightPath.length) return alert('请先确认设置并计算航线（点击侧边栏按钮）');
            startFlight();
        };
        document.getElementById('btn-stop').onclick = stopFlight;

        function addMarkerA(lat, lng) {
            markersA.forEach(m => map.removeLayer(m));
            markersA = [];
            var m = L.marker([lat, lng], {
                icon: L.divIcon({html: '<div style="font-size:20px; color:green;">🟢</div>'})
            }).addTo(map).bindPopup("起点 A");
            markersA.push(m);
        }
        function addMarkerB(lat, lng) {
            markersB.forEach(m => map.removeLayer(m));
            markersB = [];
            var m = L.marker([lat, lng], {
                icon: L.divIcon({html: '<div style="font-size:20px; color:red;">🔴</div>'})
            }).addTo(map).bindPopup("终点 B");
            markersB.push(m);
        }
        function drawRoute(route) {
            if (routePath) map.removeLayer(routePath);
            routePath = L.polyline(route, {color: 'blue', weight: 5}).addTo(map);
            map.fitBounds(routePath.getBounds().pad(0.2));
        }
        function interpolatePath(route, steps) {
            steps = steps || 15;
            var smooth = [];
            for (var i = 0; i < route.length-1; i++) {
                var lat1 = route[i][0], lng1 = route[i][1];
                var lat2 = route[i+1][0], lng2 = route[i+1][1];
                for (var s = 0; s < steps; s++) {
                    var f = s / steps;
                    smooth.push([lat1 + (lat2-lat1)*f, lng1 + (lng2-lng1)*f]);
                }
            }
            smooth.push([route[route.length-1][0], route[route.length-1][1]]);
            return smooth;
        }
        function startFlight() {
            stopFlight();
            if (!droneMarker) {
                droneMarker = L.marker(flightPath[0], {
                    icon: L.divIcon({html: '<div style="font-size:28px; color:blue;">✈️</div>'})
                }).addTo(map);
            }
            currentIndex = 0;
            flightTimer = setInterval(function() {
                if (currentIndex < flightPath.length) {
                    droneMarker.setLatLng(flightPath[currentIndex]);
                    currentIndex++;
                } else {
                    stopFlight();
                    alert('✅ 无人机已到达目的地！');
                }
            }, 40);
        }
        function stopFlight() {
            if (flightTimer) { clearInterval(flightTimer); flightTimer = null; }
        }
        function sendDataToPython() {
            var data = { a: pointA, b: pointB, obstacles: obstacles };
            window.parent.postMessage({
                type: "streamlit:setComponentValue",
                value: data
            }, "*");
        }
        console.log('🚁 地图已就绪');
    </script>
</body>
</html>
"""

html_code = html_template.replace("__INIT_DATA__", init_data_json)

# ===================== ✅ 唯一修复的地方 =====================
# 渲染地图（无返回值，只渲染）
st.components.v1.html(html_code, height=700, scrolling=False)

# 从session_state读取前端传的A/B点数据（核心修复）
returned_data = st.session_state.get("component_value")
if returned_data and isinstance(returned_data, dict):
    st.session_state.map_data = returned_data