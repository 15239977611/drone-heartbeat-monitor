import streamlit as st
import math

# 坐标转换 WGS84 -> GCJ02（已修复）
def wgs84_to_gcj02(lng, lat):
    a = 6378240.0
    ee = 0.0066934216229826
    pi = 3.141592653589793
    if abs(lng) > 180 or abs(lat) > 90:
        return lng, lat
    dLat = transformlat(lng - 105.0, lat - 35.0)
    dLng = transformlng(lng - 105.0, lat - 35.0)
    radLat = lat / 180.0 * pi
    magic = math.sin(radLat)
    magic = 1 - ee * magic * magic
    sqrtMagic = math.sqrt(magic)
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi)
    dLng = (dLng * 180.0) / (a / sqrtMagic * math.cos(radLat) * pi)
    return lng + dLng, lat + dLat

def transformlat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2 / 3
    ret += (20.0 * math.sin(lat * math.pi) + 40.0 * math.sin(lat / 3 * math.pi)) * 2 / 3
    return ret

def transformlng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat
    ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 * math.sin(2.0 * lng * math.pi)) * 2 / 3
    ret += (20.0 * math.sin(lng * math.pi) + 40.0 * math.sin(lng / 3 * math.pi)) * 2 / 3
    return ret

# 状态初始化
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "flying_flag" not in st.session_state:
    st.session_state.flying_flag = False

st.set_page_config(layout="wide")
st.title("手动绘制航线 · 无人机按手绘飞行")

# 侧边栏
with st.sidebar:
    st.subheader("🌍 坐标系")
    coord_type = st.radio("输入坐标系", ["WGS-84", "GCJ-02"], index=1)

    st.divider()
    st.subheader("📍 起点 A")
    a_lat = st.number_input("A纬度", value=32.2344, step=0.0001)
    a_lng = st.number_input("A经度", value=118.7402, step=0.0001)
    if st.button("✅ 设置A点"):
        if coord_type == "WGS-84":
            lng, lat = wgs84_to_gcj02(a_lng, a_lat)
            st.session_state.point_a = [lat, lng]
        else:
            st.session_state.point_a = [a_lat, a_lng]

    st.divider()
    st.subheader("📍 终点 B")
    b_lat = st.number_input("B纬度", value=32.2377, step=0.0001)
    b_lng = st.number_input("B经度", value=118.7411, step=0.0001)
    if st.button("✅ 设置B点"):
        if coord_type == "WGS-84":
            lng, lat = wgs84_to_gcj02(b_lng, b_lat)
            st.session_state.point_b = [lat, lng]
        else:
            st.session_state.point_b = [b_lat, b_lng]

    st.divider()
    st.subheader("✈️ 飞行控制")
    if st.button("▶️ 开始飞行"):
        st.session_state.flying_flag = True
    if st.button("⏹️ 停止飞行"):
        st.session_state.flying_flag = False

# 地图 HTML
map_html = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <style>#map { width:100%; height:720px; }</style>
</head>
<body>

<div style="margin-bottom:8px;">
    <button onclick="startDrawRoute()" style="padding:8px 14px;">开始画航线</button>
    <button onclick="clearRoute()" style="padding:8px 14px; margin-left:8px;">清空航线</button>
</div>

<div id="map"></div>

<script>
var map = L.map('map').setView([32.2344, 118.7402], 17);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    attribution: ""
}).addTo(map);

// 全局变量
let drawRouteMode = false;
let routePoints = [];
let routeMarkers = [];
let routeLine = null;

let obstacles = [];
let currentPoly = [];
let polyLayer;

// 地图点击事件
map.on("click", function(e) {
    let lat = e.latlng.lat;
    let lng = e.latlng.lng;

    if (drawRouteMode) {
        routePoints.push([lat, lng]);
        let m = L.marker([lat, lng], {
            icon: L.divIcon({
                html: '<div style="width:16px;height:16px;background:blue;border-radius:50%;border:2px solid #fff;"></div>',
                iconSize: [16, 16]
            })
        }).addTo(map);
        routeMarkers.push(m);

        if (routeLine) map.removeLayer(routeLine);
        routeLine = L.polyline(routePoints, { color: "blue", weight: 4 }).addTo(map);
        return;
    }

    currentPoly.push([lat, lng]);
    if (polyLayer) map.removeLayer(polyLayer);
    polyLayer = L.polygon(currentPoly, { color: "red" }).addTo(map);
});

// 按键功能
function startDrawRoute() {
    drawRouteMode = true;
    alert("已开启手绘航线，点击地图添加拐点");
}

function clearRoute() {
    drawRouteMode = false;
    routePoints = [];
    routeMarkers.forEach(m => map.removeLayer(m));
    routeMarkers = [];
    if (routeLine) map.removeLayer(routeLine);
    routeLine = null;
}

function saveObs() {
    if (currentPoly.length < 3) { alert("至少3个点"); return; }
    obstacles.push({ points: currentPoly });
    L.polygon(currentPoly, { color: "red", fillColor: "#ff0000", fillOpacity: 0.3 }).addTo(map);
    currentPoly = [];
    if (polyLayer) map.removeLayer(polyLayer);
}

function clearObs() {
    obstacles = [];
    location.reload();
}

// AB点
const pointA = """ + str(st.session_state.point_a) + """;
const pointB = """ + str(st.session_state.point_b) + """;

if (pointA) {
    L.marker(pointA, {
        icon: L.divIcon({
            html: '<div style="width:28px;height:28px;background:green;border-radius:50%;border:2px solid #fff;display:flex;align-items:center;justify-content:center;color:white;font-weight:bold;">A</div>',
            iconSize: [28, 28]
        })
    }).addTo(map);
}

if (pointB) {
    L.marker(pointB, {
        icon: L.divIcon({
            html: '<div style="width:28px;height:28px;background:red;border-radius:50%;border:2px solid #fff;display:flex;align-items:center;justify-content:center;color:white;font-weight:bold;">B</div>',
            iconSize: [28, 28]
        })
    }).addTo(map);
}

// 无人机飞行
let isFlying = """ + str(st.session_state.flying_flag).lower() + """;
let dronePos = pointA ? [...pointA] : null;
let drone = null;
let wayIdx = 0;

if (pointA) {
    drone = L.marker(pointA, {
        icon: L.divIcon({
            html: '<div style="font-size:28px;">✈️</div>',
            iconSize: [30, 30]
        })
    }).addTo(map);
}

function fly() {
    if (!isFlying || !dronePos || routePoints.length < 1) {
        setTimeout(fly, 50);
        return;
    }

    let target = routePoints[wayIdx];
    let dLat = target[0] - dronePos[0];
    let dLng = target[1] - dronePos[1];
    let dist = Math.hypot(dLat, dLng);

    if (dist < 0.00003) {
        wayIdx++;
        if (wayIdx >= routePoints.length) {
            isFlying = false;
            alert("已完成手绘航线飞行");
            return;
        }
    }

    dronePos[0] += dLat * 0.04;
    dronePos[1] += dLng * 0.04;

    let ang = Math.atan2(dLng, dLat) * 180 / Math.PI;
    drone.setLatLng(dronePos);
    drone.setIcon(L.divIcon({
        html: `<div style="font-size:28px;transform:rotate(${ang}deg);">✈️</div>`,
        iconSize: [30, 30]
    }));

    setTimeout(fly, 40);
}

fly();
</script>

<div style="margin-top:8px;">
    <button onclick="saveObs()" style="padding:8px 14px;">保存障碍物</button>
    <button onclick="clearObs()" style="padding:8px 14px; margin-left:8px; background:red; color:white;">清空障碍物</button>
</div>

</body>
</html>
"""

st.components.v1.html(map_html, height=800)