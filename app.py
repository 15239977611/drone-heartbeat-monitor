import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
import math
from datetime import datetime
import time
import random
import plotly.graph_objects as go
import pandas as pd

# ================== WGS84 ↔ GCJ02 坐标转换 ==================
PI = math.pi
a = 6378245.0
ee = 0.00669342162296594323

def transform_lat(x, y):
    ret = -100.0 + 2.0*x + 3.0*y + 0.2*y*y + 0.1*x*y + 0.2*math.sqrt(abs(x))
    ret += (20.0*math.sin(6.0*x*PI) + 20.0*math.sin(2.0*x*PI)) * 2.0/3.0
    ret += (20.0*math.sin(y*PI) + 40.0*math.sin(y/3.0*PI)) * 2.0/3.0
    ret += (160.0*math.sin(y/12.0*PI) + 320*math.sin(y/30.0*PI)) * 2.0/3.0
    return ret

def transform_lng(x, y):
    ret = 300.0 + x + 2.0*y + 0.1*x*x + 0.1*x*y + 0.1*math.sqrt(abs(x))
    ret += (20.0*math.sin(6.0*x*PI) + 20.0*math.sin(2.0*x*PI)) * 2.0/3.0
    ret += (20.0*math.sin(x*PI) + 40.0*math.sin(x/3.0*PI)) * 2.0/3.0
    ret += (150.0*math.sin(x/12.0*PI) + 300.0*math.sin(x/30.0*PI)) * 2.0/3.0
    return ret

def out_of_china(lat, lng):
    return not (73.66 < lng < 135.05 and 3.86 < lat < 53.55)

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lat, lng):
        return [lng, lat]
    dlat = transform_lat(lng-105.0, lat-35.0)
    dlng = transform_lng(lng-105.0, lat-35.0)
    radlat = lat/180.0*PI
    magic = math.sin(radlat)
    magic = 1 - ee*magic*magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat*180.0) / ((a*(1-ee))/(magic*sqrtmagic)*PI)
    dlng = (dlng*180.0) / (a/sqrtmagic*math.cos(radlat)*PI)
    return [lng+dlng, lat+dlat]

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lat, lng):
        return [lng, lat]
    dlat = transform_lat(lng-105.0, lat-35.0)
    dlng = transform_lng(lng-105.0, lat-35.0)
    radlat = lat/180.0*PI
    magic = math.sin(radlat)
    magic = 1 - ee*magic*magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat*180.0) / ((a*(1-ee))/(magic*sqrtmagic)*PI)
    dlng = (dlng*180.0) / (a/sqrtmagic*math.cos(radlat)*PI)
    return [lng*2-(lng+dlng), lat*2-(lat+dlat)]

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
    st.session_state.drone_height = 8

if "drone_heartbeat" not in st.session_state:
    st.session_state.drone_heartbeat = {
        "last_time": datetime.now(), "signal_strength":95, "battery":88,
        "gps_status":"正常", "flight_status":"待命", "latitude":32.2330, "longitude":118.7490,
        "speed":0.0, "heartbeat_interval":1, "heartbeat_seq":0
    }
if "heartbeat_log" not in st.session_state:
    st.session_state.heartbeat_log = []
if "heartbeat_chart_data" not in st.session_state:
    st.session_state.heartbeat_chart_data = {"time":[],"seq":[]}
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False

if "coord_system" not in st.session_state:
    st.session_state.coord_system = "WGS84"
if "transformed_points" not in st.session_state:
    st.session_state.transformed_points = {"point_a":None,"point_b":None,"obstacles":[]}

GROUND_HEIGHT = 0
SAFE_OFFSET = 0.00015
REAL_WORLD_HEIGHTS = {
    "自定义障碍物":50,"普通房屋":20,"高层楼房":80,"大树/电线杆":10,
    "操场/空地":0,"桥梁/高架":15,"塔楼/信号塔":60
}

# ================== 存储 ==================
def save_all():
    with open("geo_obstacles.json","w",encoding="utf-8") as f:
        json.dump({"obs":st.session_state.obstacles_all,"types":st.session_state.obstacles_type,"heights":st.session_state.obstacles_height},f)

def load_all():
    try:
        with open("geo_obstacles.json","r",encoding="utf-8") as f:
            d=json.load(f)
            st.session_state.obstacles_all=d.get("obs",[])
            st.session_state.obstacles_type=d.get("types",[])
            st.session_state.obstacles_height=d.get("heights",[])
    except:
        st.session_state.obstacles_all=[]
        st.session_state.obstacles_type=[]
        st.session_state.obstacles_height=[]

load_all()
st.set_page_config(page_title="最短避障航线",layout="wide")

# ================== 核心：最短避障航线 ==================
def distance(p1,p2):
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def calculate_shortest_avoid_route():
    A = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    B = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    if not A or not B:
        return [], "未设置起点A/终点B"

    drone_h = st.session_state.drone_height
    obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    high_obstacles = []

    for i, pts in enumerate(obstacles):
        if len(pts)<3: continue
        h = st.session_state.obstacles_height[i] if i<len(st.session_state.obstacles_height) else 50
        if h > drone_h:
            poly = Polygon(pts)
            cx, cy = poly.centroid.x, poly.centroid.y
            safe_poly = []
            for (x,y) in poly.exterior.coords[:-1]:
                dx = x-cx
                dy = y-cy
                dd = math.hypot(dx,dy) or 1
                sx = x + dx/dd * SAFE_OFFSET
                sy = y + dy/dd * SAFE_OFFSET
                safe_poly.append((sx,sy))
            high_obstacles.append(Polygon(safe_poly))

    if not high_obstacles:
        return [A,B], f"🟢 直线最短航线（无人机高度{drone_h}m）"

    # 基础直线
    path = [A, B]
    # 多次迭代修正，保证最短绕切
    for _ in range(3):
        new_path = [path[0]]
        for i in range(1, len(path)):
            p_prev = new_path[-1]
            p_curr = path[i]
            line = LineString([p_prev, p_curr])
            hit = None
            for poly in high_obstacles:
                if line.intersects(poly):
                    hit = poly
                    break
            if hit is None:
                new_path.append(p_curr)
                continue
            # 找最短切线点
            near_p, _ = nearest_points(line, hit.exterior)
            cx, cy = hit.centroid.x, hit.centroid.y
            dx = near_p.x - cx
            dy = near_p.y - cy
            dd = math.hypot(dx, dy) or 1
            side_pt = (near_p.x + dx/dd*0.00025, near_p.y + dy/dd*0.00025)
            new_path.append(side_pt)
            new_path.append(p_curr)
        path = new_path

    # 去重
    clean = []
    seen = set()
    for p in path:
        t = (round(p[0],8), round(p[1],8))
        if t not in seen:
            seen.add(t)
            clean.append(p)
    return clean, f"🔴 最短避障航线（不重叠，绕高障碍物）"

# ================== 心跳 ==================
def update_heartbeat():
    if not st.session_state.heartbeat_running: return
    now = datetime.now()
    if (now - st.session_state.drone_heartbeat["last_time"]).total_seconds() < 1:
        return
    st.session_state.drone_heartbeat["heartbeat_seq"] +=1
    sig = st.session_state.drone_heartbeat["signal_strength"] + random.randint(-2,2)
    st.session_state.drone_heartbeat["signal_strength"] = max(80,min(100,sig))
    if random.randint(1,5)==3:
        st.session_state.drone_heartbeat["battery"] = max(0, st.session_state.drone_heartbeat["battery"]-1)
    st.session_state.drone_heartbeat["last_time"] = now
    st.session_state.heartbeat_log.append({
        "time":now.strftime("%H:%M:%S"),
        "seq":st.session_state.drone_heartbeat["heartbeat_seq"]
    })
    if len(st.session_state.heartbeat_log)>30:
        st.session_state.heartbeat_log.pop(0)

def draw_heart():
    df = pd.DataFrame(st.session_state.heartbeat_log)
    fig = go.Figure()
    if not df.empty:
        fig.add_trace(go.Scatter(x=df["time"],y=df["seq"],mode="lines+markers"))
    fig.update_layout(height=300,title="心跳")
    return fig

# ================== 侧边栏 ==================
with st.sidebar:
    st.title("无人机最短避障系统")
    page = st.radio("页面",["航线规划","飞行监控"])
    if page=="航线规划":
        st.subheader("坐标系")
        st.session_state.coord_system = st.selectbox("目标",["WGS84","GCJ02"])
        if st.button("✅ 确认转换"):
            # A
            if st.session_state.point_a:
                lat,lng = st.session_state.point_a
                if st.session_state.coord_system=="GCJ02":
                    nlng,nlat = wgs84_to_gcj02(lng,lat)
                else:
                    nlng,nlat = gcj02_to_wgs84(lng,lat)
                st.session_state.transformed_points["point_a"] = [round(nlat,6),round(nlng,6)]
            # B
            if st.session_state.point_b:
                lat,lng = st.session_state.point_b
                if st.session_state.coord_system=="GCJ02":
                    nlng,nlat = wgs84_to_gcj02(lng,lat)
                else:
                    nlng,nlat = gcj02_to_wgs84(lng,lat)
                st.session_state.transformed_points["point_b"] = [round(nlat,6),round(nlng,6)]
            # 障碍物
            nobs = []
            for obs in st.session_state.obstacles_all:
                ps = []
                for lat,lng in obs:
                    if st.session_state.coord_system=="GCJ02":
                        nlng,nlat = wgs84_to_gcj02(lng,lat)
                    else:
                        nlng,nlat = gcj02_to_wgs84(lng,lat)
                    ps.append([round(nlat,6),round(nlng,6)])
                nobs.append(ps)
            st.session_state.transformed_points["obstacles"] = nobs
            st.success("转换完成")
        if st.button("🔄重置原始坐标"):
            st.session_state.transformed_points = {"point_a":None,"point_b":None,"obstacles":[]}

    st.subheader("飞行高度")
    st.session_state.drone_height = st.slider("米",0,200,8)

    st.subheader("圈选障碍物")
    dtype = st.selectbox("类型",["无","自定义障碍物","普通房屋","高层楼房","大树/电线杆","操场/空地","桥梁/高架","塔楼/信号塔"])
    c1,c2 = st.columns(2)
    with c1:
        if st.button("开始圈选") and dtype!="无":
            st.session_state.drawing_mode = dtype
            st.session_state.current_points=[]
    with c2:
        if st.button("完成圈选") and st.session_state.drawing_mode:
            if len(st.session_state.current_points)>=3:
                ps = st.session_state.current_points
                if ps[0]!=ps[-1]:
                    ps.append(ps[0])
                st.session_state.obstacles_all.append(ps)
                st.session_state.obstacles_type.append(st.session_state.drawing_mode)
                st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[dtype])
                save_all()
            st.session_state.drawing_mode=None
            st.session_state.current_points=[]
    if st.button("清空障碍物"):
        st.session_state.obstacles_all=[]
        st.session_state.obstacles_type=[]
        st.session_state.obstacles_height=[]
        save_all()

    st.subheader("A/B点")
    c1,c2=st.columns(2)
    with c1:
        st.write("A点")
        if st.session_state.transformed_points["point_a"]:
            st.success(f"{st.session_state.transformed_points['point_a']}")
        elif st.session_state.point_a:
            st.success(f"{st.session_state.point_a}")
        if st.button("清A"):
            st.session_state.point_a=None
            st.session_state.transformed_points["point_a"]=None
    with c2:
        st.write("B点")
        if st.session_state.transformed_points["point_b"]:
            st.success(f"{st.session_state.transformed_points['point_b']}")
        elif st.session_state.point_b:
            st.success(f"{st.session_state.point_b}")
        if st.button("清B"):
            st.session_state.point_b=None
            st.session_state.transformed_points["point_b"]=None

# ================== 航线页面 ==================
if page=="航线规划":
    st.title("最短避障航线（绝不重叠）")
    route,info = calculate_shortest_avoid_route()
    st.info(info)
    center = [32.2330, 118.7490]
    if st.session_state.transformed_points["point_a"]:
        center = st.session_state.transformed_points["point_a"]
    elif st.session_state.point_a:
        center = st.session_state.point_a
    # 修复：添加 attr 参数（瓦片归属）
    m = folium.Map(
        location=center, 
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri, DigitalGlobe, GeoEye, Earthstar Geographics, CNES/Airbus DS, USDA, USGS, AeroGRID, IGN, and the GIS User Community"
    )
    # A
    a = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    b = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    if a:
        folium.CircleMarker(a, color="green", radius=10, fill=True).add_to(m)
    if b:
        folium.CircleMarker(b, color="red", radius=10, fill=True).add_to(m)
    # 障碍物
    obs = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    for o in obs:
        if len(o)>2:
            folium.Polygon(locations=o, color="red", fill=True, fill_opacity=0.3).add_to(m)
    # 航线
    if len(route)>=2:
        folium.PolyLine(route, color="blue", weight=8, opacity=0.9).add_to(m)
        for p in route[1:-1]:
            folium.CircleMarker(p, color="yellow", radius=6, fill=True).add_to(m)
    # 绘制中
    if st.session_state.drawing_mode and st.session_state.current_points:
        folium.PolyLine(st.session_state.current_points, color="orange", weight=5).add_to(m)
    out = st_folium(m, height=800)
    if out and out.get("last_clicked"):
        lat=out["last_clicked"]["lat"]
        lng=out["last_clicked"]["lng"]
        if st.session_state.drawing_mode:
            st.session_state.current_points.append([lat,lng])
        else:
            if not st.session_state.point_a:
                st.session_state.point_a=(lat,lng)
                st.session_state.transformed_points["point_a"]=None
            elif not st.session_state.point_b:
                st.session_state.point_b=(lat,lng)
                st.session_state.transformed_points["point_b"]=None

# ================== 监控页面 ==================
else:
    st.title("飞行监控")
    c1,c2,c3=st.columns(3)
    with c1:
        if st.button("开始心跳"):
            st.session_state.heartbeat_running=True
    with c2:
        if st.button("停止心跳"):
            st.session_state.heartbeat_running=False
    with c3:
        if st.button("清空日志"):
            st.session_state.heartbeat_log=[]
    update_heartbeat()
    st.plotly_chart(draw_heart(), use_container_width=True)
    st.metric("信号",f"{st.session_state.drone_heartbeat['signal_strength']}%")
    st.metric("电量",f"{st.session_state.drone_heartbeat['battery']}%")
    for log in reversed(st.session_state.heartbeat_log[-10:]):
        st.text(f"{log['time']} 心跳 {log['seq']}")
    if st.session_state.heartbeat_running:
        time.sleep(1)
        st.rerun()