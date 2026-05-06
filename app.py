import streamlit as st
import json
from shapely.geometry import Polygon, LineString
from shapely.ops import nearest_points
from shapely.affinity import scale
import math

# ---------- 初始化 ----------
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
if "map_data" not in st.session_state:
    st.session_state.map_data = {"a": None, "b": None, "obstacles": []}

REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50, "普通房屋": 20, "高层楼房": 80,
    "大树/电线杆": 10, "操场/空地": 0, "桥梁/高架": 15, "塔楼/信号塔": 60
}

# ---------- 绕飞算法 (与之前相同) ----------
def calculate_route(A, B, obstacles_all, heights, drone_h, safety_r, direction):
    OFFSET = safety_r / 111000.0
    route = [A]

    for i, coords in enumerate(obstacles_all):
        if len(coords) < 3:
            continue
        h = heights[i]
        if h <= drone_h:
            continue
        try:
            poly = Polygon(coords)
            safe_poly = scale(poly, 1 + safety_r/1600, 1 + safety_r/1600, origin='centroid')
            cx, cy = poly.centroid.x, poly.centroid.y
            line = LineString([route[-1], B])
            if not line.intersects(safe_poly):
                continue
            p, _ = nearest_points(line, safe_poly.boundary)
            px, py = p.x, p.y
            dx, dy = px - cx, py - cy
            dist = math.hypot(dx, dy) or 1
            dx, dy = dx/dist, dy/dist
            if direction == "左":
                wx, wy = -dy, dx
            else:
                wx, wy = dy, -dx
            route.append((px + wx * OFFSET, py + wy * OFFSET))
        except:
            continue
    route.append(B)
    return route

# ---------- 生成插值路径 ----------
def interpolate_path(path, steps=15):
    smooth = []
    for i in range(len(path)-1):
        lat1, lng1 = path[i]
        lat2, lng2 = path[i+1]
        for s in range(steps):
            f = s / steps
            lat = lat1 + (lat2 - lat1) * f
            lng = lng1 + (lng2 - lng1) * f
            smooth.append([lat, lng])
    smooth.append([path[-1][0], path[-1][1]])
    return smooth

# ---------- 侧边栏 ----------
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

    st.markdown("---")
    st.write("在地图上点击设置 A/B，点击「绘制障碍物」后开始圈选。")
    st.write("完成后点击下方按钮，将数据提交并计算航线。")

    # 这个按钮是唯一会触发地图重绘的地方
    if st.button("📐 确认设置并计算航线"):
        # 从 map_data 读取用户在地图上设置的数据
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
            st.session_state.obstacles_height = [obs_height] * len(obs)  # 所有障碍物暂时用统一高度
            # 计算航线
            route = calculate_route(
                A, B,
                st.session_state.obstacles_all,
                st.session_state.obstacles_height,
                st.session_state.drone_height,
                st.session_state.drone_safety_radius,
                st.session_state.avoid_direction
            )
            st.session_state.route = route
        else:
            st.warning("请先在地图上设置起点 A 和终点 B")

# ---------- 构造传给 HTML 的数据 ----------
route_js = json.dumps(st.session_state.route)

# 把需要的初始数据序列化给 JS
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

# ---------- 内嵌 HTML / JS 地图 ----------
html_code = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>
        #map {{
            height: 100vh;
            width: 100%;
        }}
        .btn-container {{
            position: absolute;
            top: 10px; left: 10px; z-index: 1000;
            display: flex; gap: 5px; flex-wrap: wrap;
        }}
        .btn-container button {{
            background: white; border: 1px solid #ccc; border-radius: 4px;
            padding: 6px 12px; cursor: pointer; font-size: 14px;
        }}
        .btn-container button.active {{
            background: #4CAF50; color: white; border-color: #4CAF50;
        }}
    </style>
</head>
<body style="margin:0">
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
        // 初始化地图
        var map = L.map('map').setView([32.2330, 118.7490], 18);
        L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
            attribution: '© OpenStreetMap contributors'
        }}).addTo(map);

        // 存储用户交互数据
        var pointA = null;
        var pointB = null;
        var tempObsPoints = [];        // 正在绘制中的多边形点
        var obstacles = [];            // 已完成的障碍物，每个元素是 [[lat,lng],...]
        var markersA = [];
        var markersB = [];
        var drawMode = false;          // 是否在绘制障碍物
        var drawLayer = null;          // 预览用

        // 飞行相关
        var droneMarker = null;
        var routePath = null;
        var flightTimer = null;
        var flightPath = [];
        var currentIndex = 0;

        // 接收 Python 传来的数据
        var initData = {init_data_json};

        // 初始化加载已有的数据（从 Python 端）
        if (initData.a) {{
            pointA = initData.a;
            addMarkerA(pointA[0], pointA[1]);
        }}
        if (initData.b) {{
            pointB = initData.b;
            addMarkerB(pointB[0], pointB[1]);
        }}
        if (initData.obstacles && initData.obstacles.length) {{
            obstacles = initData.obstacles;
            drawAllObstacles();
        }}
        if (initData.route && initData.route.length >= 2) {{
            drawRoute(initData.route);
            flightPath = interpolatePath(initData.route);
        }}

        // ----- 地图事件 -----
        map.on('click', function(e) {{
            if (drawMode) {{
                tempObsPoints.push([e.latlng.lat, e.latlng.lng]);
                updateObsPreview();
            }}
        }});

        // ----- 按钮逻辑 -----
        document.getElementById('btn-set-a').addEventListener('click', function() {{
            if (drawMode) return alert('请先完成障碍物绘制');
            map.once('click', function(e) {{
                pointA = [e.latlng.lat, e.latlng.lng];
                addMarkerA(pointA[0], pointA[1]);
                sendDataToPython();
            }});
        }});

        document.getElementById('btn-set-b').addEventListener('click', function() {{
            if (drawMode) return alert('请先完成障碍物绘制');
            map.once('click', function(e) {{
                pointB = [e.latlng.lat, e.latlng.lng];
                addMarkerB(pointB[0], pointB[1]);
                sendDataToPython();
            }});
        }});

        document.getElementById('btn-draw-obs').addEventListener('click', function() {{
            drawMode = true;
            tempObsPoints = [];
            if (drawLayer) map.removeLayer(drawLayer);
            drawLayer = L.polyline([], {{color: 'red', dashArray: '5, 5'}}).addTo(map);
            document.getElementById('btn-draw-obs').classList.add('active');
            alert('现在点击地图添加障碍物顶点，完成后按「完成障碍物」');
        }});

        document.getElementById('btn-finish-obs').addEventListener('click', function() {{
            if (!drawMode) return alert('请先点击「绘制障碍物」');
            if (tempObsPoints.length < 3) return alert('至少需要3个点');
            obstacles.push(tempObsPoints.slice());
            drawAllObstacles();
            tempObsPoints = [];
            drawMode = false;
            if (drawLayer) map.removeLayer(drawLayer);
            document.getElementById('btn-draw-obs').classList.remove('active');
            sendDataToPython();
        }});

        document.getElementById('btn-clear-obs').addEventListener('click', function() {{
            obstacles = [];
            drawAllObstacles();
            sendDataToPython();
        }});

        document.getElementById('btn-fly').addEventListener('click', function() {{
            if (!flightPath.length) return alert('请先确认设置并计算航线（点击侧边栏按钮）');
            startFlight();
        }});

        document.getElementById('btn-stop').addEventListener('click', function() {{
            stopFlight();
        }});

        // ----- 辅助函数 -----
        function addMarkerA(lat, lng) {{
            markersA.forEach(m => map.removeLayer(m));
            markersA = [];
            var icon = L.divIcon({{html: '<div style="font-size:20px; color:green;">🟢</div>', className: 'a-marker'}});
            var m = L.marker([lat, lng], {{icon: icon, draggable: false}}).addTo(map).bindPopup("起点 A");
            markersA.push(m);
        }}

        function addMarkerB(lat, lng) {{
            markersB.forEach(m => map.removeLayer(m));
            markersB = [];
            var icon = L.divIcon({{html: '<div style="font-size:20px; color:red;">🔴</div>', className: 'b-marker'}});
            var m = L.marker([lat, lng], {{icon: icon, draggable: false}}).addTo(map).bindPopup("终点 B");
            markersB.push(m);
        }}

        function updateObsPreview() {{
            if (drawLayer) {{
                drawLayer.setLatLngs(tempObsPoints);
            }}
        }}

        function drawAllObstacles() {{
            // 移除已存在的障碍物图层（简单做法：重新构建即可）
            map.eachLayer(function(layer) {{
                if (layer instanceof L.Polygon && layer.options.color === 'orange') {{
                    map.removeLayer(layer);
                }}
            }});
            obstacles.forEach(function(pts) {{
                L.polygon(pts, {{color: 'orange', fillOpacity: 0.4}}).addTo(map);
            }});
        }}

        function drawRoute(route) {{
            if (routePath) map.removeLayer(routePath);
            routePath = L.polyline(route, {{color: 'blue', weight: 5, opacity: 0.9}}).addTo(map);
            // 适配边界
            map.fitBounds(routePath.getBounds().pad(0.2));
        }}

        function interpolatePath(route, steps=15) {{
            var smooth = [];
            for (var i = 0; i < route.length-1; i++) {{
                var lat1 = route[i][0], lng1 = route[i][1];
                var lat2 = route[i+1][0], lng2 = route[i+1][1];
                for (var s = 0; s < steps; s++) {{
                    var f = s / steps;
                    smooth.push([lat1 + (lat2 - lat1) * f, lng1 + (lng2 - lng1) * f]);
                }}
            }}
            smooth.push([route[route.length-1][0], route[route.length-1][1]]);
            return smooth;
        }}

        function startFlight() {{
            stopFlight();
            if (!droneMarker) {{
                var icon = L.divIcon({{html: '<div style="font-size:28px; color:blue;">✈️</div>'}});
                droneMarker = L.marker(flightPath[0], {{icon: icon}}).addTo(map);
            }}
            currentIndex = 0;
            flightTimer = setInterval(function() {{
                if (currentIndex < flightPath.length) {{
                    droneMarker.setLatLng(flightPath[currentIndex]);
                    currentIndex++;
                }} else {{
                    stopFlight();
                    alert('✅ 无人机已到达目的地！');
                }}
            }}, 40);
        }}

        function stopFlight() {{
            if (flightTimer) {{
                clearInterval(flightTimer);
                flightTimer = null;
            }}
        }}

        function sendDataToPython() {{
            // 更新 Python 端的 map_data（不触发 Python 计算，只是保存状态）
            var data = {{
                a: pointA,
                b: pointB,
                obstacles: obstacles
            }};
            window.parent.postMessage({{
                type: "streamlit:setComponentValue",
                value: data
            }}, "*");
        }}
    </script>
</body>
</html>
"""

# ---------- 显示地图组件 ----------
map_output = st.components.v1.html(
    html_code,
    height=700,
    scrolling=False
)

# 在这里接收来自 JS 的数据（只用于保存，不自动重绘地图）
if "map_data_receiver" not in st.session_state:
    st.session_state.map_data_receiver = None

# 由于 st.components.v1.html 的返回值是通过 on_change 来的，我们直接使用 st.experimental_get_query_params 之类比较麻烦。
# 这里使用一个技巧：我们不需要自动获取返回值，因为我们仅在点击“确认设置”时才需要数据。
# 点击“确认设置”按钮时，直接从之前累积的 map_data（通过 JS 发送时会自动更新 session_state）
# 但 st.components.v1.html 的返回值需要设置 key 和 on_change，这里为了简单，我们换一种更可靠的方式：
# 我们可以用一个 callback 来捕获 JS 传回的数据。
# 但简化起见，我们让用户必须点击侧边栏的按钮，那时数据已经通过 JS 的 sendDataToPython 存在于 streamlit 的 session 中吗？
# 是的，如果组件正确返回了数据，我们可以读取它。

# 我们需要处理组件返回值：
if "map_data" not in st.session_state:
    st.session_state.map_data = {"a": None, "b": None, "obstacles": []}

# 捕获组件回传（st.components.v1.html 支持 on_change 回调）
def update_map_data(new_value):
    st.session_state.map_data = new_value

# 重新渲染时需要用 key，并绑定 on_change
# 注意：每次 rerun 会重新创建组件，但如果 key 不变且 html 内容没变，组件不会重新执行 JS。
# 为了在数据不变时不重建，我们把 html 内容用 key 固定。
# 但这里 html_code 依赖 init_data_json，而 init_data_json 在未确认时不变，所以地图不会重绘。
st.components.v1.html(
    html_code,
    height=700,
    scrolling=False,
    key="leaflet_map",
    on_change=update_map_data
)