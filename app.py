import streamlit as st
import leafmap.foliumap as leafmap
import json
import math
import time
from datetime import datetime
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
from shapely.affinity import scale

# ================== 坐标系转换 ==================
PI = math.pi
a = 6378245.0
ee = 0.00669342162296594323

def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    return ret

def transform_lng(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    return ret

def out_of_china(lat, lng):
    return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lat, lng): return [lng, lat]
    dlat = transform_lat(lng-105, lat-35)
    dlng = transform_lng(lng-105, lat-35)
    rad = lat / 180 * PI
    magic = math.sin(rad)
    return [lng+dlng, lat+dlat]

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lat, lng): return [lng, lat]
    dlat = transform_lat(lng-105, lat-35)
    dlng = transform_lng(lng-105, lat-35)
    return [lng*2-(lng+dlng), lat*2-(lat+dlat)]

# ================== 初始化 ==================
st.set_page_config(layout="wide")
for k in ["point_a","point_b","obstacles_all","obstacles_type","obstacles_height",
           "drone_height","drone_safety_radius","avoid_direction","flight_path",
           "drone_pos","is_flying","drawing_mode","current_points"]:
    if k not in st.session_state:
        st.session_state[k] = None if k in ["point_a","point_b","drone_pos"] else [] if k=="obstacles_all" else 8 if k=="drone_height" else 15 if k=="drone_safety_radius" else "自动" if k=="avoid_direction" else False if k=="is_flying" else None if k=="drawing_mode" else []

REAL_WORLD_HEIGHTS = {"自定义障碍物":50,"普通房屋":20,"高层楼房":80,"大树/电线杆":10,"操场/空地":0,"桥梁/高架":15,"塔楼/信号塔":60}

# ================== 平滑轨迹 ==================
def interpolate_path(path, steps=50):
    out = []
    for i in range(len(path)-1):
        lat1,lng1 = path[i]
        lat2,lng2 = path[i+1]
        for s in range(steps):
            f = s/steps
            out.append((lat1+(lat2-lat1)*f, lng1+(lng2-lng1)*f))
    return out

# ================== 航线算法 ==================
def calc_route():
    A = st.session_state.point_a
    B = st.session_state.point_b
    if not A or not B: return []
    h = st.session_state.drone_height
    r = st.session_state.drone_safety_radius
    dir = st.session_state.avoid_direction
    route = [A]
    for coords in st.session_state.obstacles_all:
        if len(coords)<3: continue
        idx = st.session_state.obstacles_all.index(coords)
        oh = st.session_state.obstacles_height[idx] if idx<len(st.session_state.obstacles_height) else 50
        if oh <= h: continue
        try:
            p = Polygon(coords)
            s = scale(p, xfact=1+r/1600, yfact=1+r/1600, origin='centroid')
            cx,cy = p.centroid.x, p.centroid.y
            line = LineString([route[-1], B])
            if not line.intersects(s): continue
            px,py = nearest_points(line, s.boundary)[0].coords[0]
            dx,dy = px-cx, py-cy
            d = math.hypot(dx,dy) or 1
            dx/=d; dy/=d
            if dir=="左": ox,oy = -dy, dx
            elif dir=="右": ox,oy = dy, -dx
            else: ox,oy=dx,dy
            route.append((px+ox*r/12000, py+oy*r/12000))
        except: continue
    route.append(B)
    return route

# ================== 地图 ==================
with st.sidebar:
    st.title("✈️ 无人机避障")
    page = st.radio("页面", ["航线规划", "飞行监控"])
    
    st.subheader("🛸 高度")
    st.session_state.drone_height = st.slider("米",0,200,8)
    
    st.subheader("🛡️ 安全半径")
    st.session_state.drone_safety_radius = st.slider("米",1,50,15)
    
    st.subheader("↔️ 绕飞方向")
    c1,c2 = st.columns(2)
    with c1:
        if st.button("⬅️ 向左"): st.session_state.avoid_direction="左"
    with c2:
        if st.button("➡️ 向右"): st.session_state.avoid_direction="右"
    st.info(f"当前：{st.session_state.avoid_direction}")
    
    st.subheader("✈️ 飞行")
    c1,c2 = st.columns(2)
    with c1:
        if st.button("▶️ 起飞"):
            route = calc_route()
            if len(route)>=2:
                st.session_state.flight_path = interpolate_path(route)
                st.session_state.is_flying = True
                st.session_state.drone_pos = st.session_state.flight_path[0]
    with c2:
        if st.button("⏹️ 停止"):
            st.session_state.is_flying=False
    
    st.subheader("🌍 障碍物")
    t = st.selectbox("类型", ["无"]+list(REAL_WORLD_HEIGHTS.keys()))
    if st.button("🟢 开始圈选"):
        st.session_state.drawing_mode = t
        st.session_state.current_points=[]
    if st.button("✅ 完成圈选") and st.session_state.drawing_mode and len(st.session_state.current_points)>=3:
        st.session_state.current_points.append(st.session_state.current_points[0])
        st.session_state.obstacles_all.append(st.session_state.current_points)
        st.session_state.obstacles_type.append(st.session_state.drawing_mode)
        st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[t])
    if st.button("🗑️ 清空"):
        st.session_state.obstacles_all=[]
    
    st.subheader("📍 A / B")
    if st.button("🟢 设置A点"): st.session_state.point_a=None
    if st.button("🔴 设置B点"): st.session_state.point_b=None

if page == "航线规划":
    m = leafmap.Map(location=[32.2330, 118.7490], zoom=18, tiles="ESRI Satellite")
    route = calc_route()

    if st.session_state.point_a:
        m.add_marker([st.session_state.point_a[0], st.session_state.point_a[1]], icon="home", color="blue")
    if st.session_state.point_b:
        m.add_marker([st.session_state.point_b[0], st.session_state.point_b[1]], icon="flag", color="red")
    
    for o in st.session_state.obstacles_all:
        if len(o)>2:
            m.add_polygon(o, color="orange", fill_color="orange", fill_opacity=0.4)
    
    if len(route)>=2:
        m.add_polyline(route, color="blue", weight=5)

    # ✅ 核心：永不闪烁！
    if st.session_state.drone_pos:
        lat,lng = st.session_state.drone_pos
        m.add_marker([lat,lng], icon="plane", color="blue", rotation=0)

    m.to_streamlit(height=700)

    # ✅ 无刷新动画
    if st.session_state.is_flying and st.session_state.flight_path:
        i = st.session_state.get("flight_i",0)
        if i < len(st.session_state.flight_path):
            st.session_state.drone_pos = st.session_state.flight_path[i]
            st.session_state.flight_i = i+1
            time.sleep(0.05)
            st.rerun()
        else:
            st.session_state.is_flying=False
            st.success("到达目的地！")

else:
    st.title("📡 飞行监控")
    st.write("飞行中：", st.session_state.is_flying)
    st.write("无人机位置：", st.session_state.drone_pos)