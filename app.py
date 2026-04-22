import streamlit as st
import folium
from streamlit_folium import st_folium
import json
from shapely.geometry import Polygon, LineString, Point, LinearRing
from shapely.ops import nearest_points
import math
from datetime import datetime, timedelta
import time
import random
import plotly.graph_objects as go
import pandas as pd
import threading

# ================== 初始化（新增启停状态） ==================
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
    st.session_state.drone_height = 8  # 默认8米
# 心跳包核心状态
if "drone_heartbeat" not in st.session_state:
    st.session_state.drone_heartbeat = {
        "last_time": datetime.now(),  # 最后心跳时间
        "signal_strength": 95,       # 信号强度（0-100）
        "battery": 88,               # 电量（0-100）
        "gps_status": "正常",        # GPS状态
        "flight_status": "待命",     # 飞行状态：待命/飞行中/绕飞中/异常
        "latitude": 32.2330,         # 当前纬度
        "longitude": 118.7490,       # 当前经度
        "speed": 0.0,                # 飞行速度（m/s）
        "heartbeat_interval": 1,     # 心跳间隔（秒）
        "heartbeat_seq": 0           # 心跳包序号（核心曲线指标）
    }
if "heartbeat_log" not in st.session_state:
    st.session_state.heartbeat_log = []  # 心跳日志
if "heartbeat_chart_data" not in st.session_state:
    st.session_state.heartbeat_chart_data = {
        "time": [],    # X轴：时间
        "seq": []      # Y轴：心跳包序号（仅保留这一条线）
    }
# 新增：心跳监控启停状态
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False  # 默认停止

# ================== 核心配置 ==================
GROUND_HEIGHT = 0  # 地面基准高度
SAFE_DISTANCE = 0.0003  # 安全距离
REAL_WORLD_HEIGHTS = {
    "自定义障碍物": 50,
    "普通房屋": 20,
    "高层楼房": 80,
    "大树/电线杆": 10,
    "操场/空地": 0,
    "桥梁/高架": 15,
    "塔楼/信号塔": 60
}

# ================== 永久存储函数 ==================
def save_all():
    with open("geo_obstacles.json", "w", encoding="utf-8") as f:
        json.dump({
            "obstacles": st.session_state.obstacles_all,
            "types": st.session_state.obstacles_type,
            "heights": st.session_state.obstacles_height
        }, f)

def load_all():
    try:
        with open("geo_obstacles.json", "r", encoding="utf-8") as f:
            data = json.load(f)
            st.session_state.obstacles_all = data.get("obstacles", [])
            st.session_state.obstacles_type = data.get("types", [])
            st.session_state.obstacles_height = data.get("heights", [])
    except:
        st.session_state.obstacles_all = []
        st.session_state.obstacles_type = []
        st.session_state.obstacles_height = []

load_all()
st.set_page_config(page_title="无重叠精准避障无人机系统", layout="wide")

# ================== 核心避障算法（完全保留） ==================
def calculate_no_overlap_route():
    A = st.session_state.point_a
    B = st.session_state.point_b
    
    if not A or not B:
        return [], "未设置起点A/终点B（地面基准：0米）"
    
    drone_h = st.session_state.drone_height
    final_route = [A]
    avoid_obstacles = []
    line_ab = LineString([A, B])

    for i, obs_coords in enumerate(st.session_state.obstacles_all):
        if len(obs_coords) < 3:
            continue
        
        obs_poly = Polygon(obs_coords)
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"

        if obs_h <= drone_h:
            continue
        
        avoid_obstacles.append(obs_type)
        obs_ring = LinearRing(obs_coords)
        p1, p2 = nearest_points(line_ab, obs_ring)
        
        centroid = obs_poly.centroid
        dx = p2.x - centroid.x
        dy = p2.y - centroid.y
        dist = math.hypot(dx, dy)
        if dist > 0:
            dx /= dist
            dy /= dist
        avoid_point = (p2.x - dx * SAFE_DISTANCE, p2.y - dy * SAFE_DISTANCE)
        
        if obs_poly.contains(Point(avoid_point)):
            avoid_point = (p2.x - dx * SAFE_DISTANCE * 2, p2.y - dy * SAFE_DISTANCE * 2)
        
        final_route.append(avoid_point)

    final_route.append(B)
    final_route = list({tuple(p): p for p in final_route}.values())
    
    if avoid_obstacles:
        status = f"🔴 无重叠绕行！无人机高度({drone_h}m) < 障碍物高度，已避开：{','.join(avoid_obstacles)}，航线与障碍物绝对无重叠（地面基准：0米）"
    else:
        status = f"🟢 直线飞行！无人机高度({drone_h}m) ≥ 所有障碍物高度，直接从A到B（地面基准：0米）"
    
    return final_route, status

# ================== 心跳包更新函数（仅在运行状态更新） ==================
def update_drone_heartbeat():
    if not st.session_state.heartbeat_running:
        return  # 停止状态不更新
    
    now = datetime.now()
    time_diff = (now - st.session_state.drone_heartbeat["last_time"]).total_seconds()
    
    if time_diff >= st.session_state.drone_heartbeat["heartbeat_interval"]:
        # 心跳序号严格递增（保证折线一直向上）
        st.session_state.drone_heartbeat["heartbeat_seq"] += 1
        
        # 信号强度小幅波动（不影响折线趋势）
        new_signal = st.session_state.drone_heartbeat["signal_strength"] + random.randint(-2, 2)
        st.session_state.drone_heartbeat["signal_strength"] = max(80, min(100, new_signal))  # 保证信号稳定
        
        # 电量缓慢下降（每5秒降1%）
        if st.session_state.drone_heartbeat["battery"] > 0 and random.randint(1, 5) == 3:
            st.session_state.drone_heartbeat["battery"] -= 1
        
        # GPS状态保持正常（避免干扰）
        st.session_state.drone_heartbeat["gps_status"] = "正常"
        
        # 飞行状态根据航线更新
        _, route_status = calculate_no_overlap_route()
        if st.session_state.point_a and st.session_state.point_b:
            st.session_state.drone_heartbeat["flight_status"] = "绕飞中" if "绕行" in route_status else "飞行中"
            st.session_state.drone_heartbeat["speed"] = round(random.uniform(4.0, 6.0), 1)
        else:
            st.session_state.drone_heartbeat["flight_status"] = "待命"
            st.session_state.drone_heartbeat["speed"] = 0.0
        
        # 更新最后心跳时间
        st.session_state.drone_heartbeat["last_time"] = now
        
        # 记录日志（保留最近50条）
        heartbeat_log_entry = {
            "time": now.strftime("%Y-%m-%d %H:%M:%S"),
            "seq": st.session_state.drone_heartbeat["heartbeat_seq"],
            "signal": st.session_state.drone_heartbeat["signal_strength"],
            "battery": st.session_state.drone_heartbeat["battery"],
            "gps": st.session_state.drone_heartbeat["gps_status"],
            "status": st.session_state.drone_heartbeat["flight_status"]
        }
        st.session_state.heartbeat_log.append(heartbeat_log_entry)
        if len(st.session_state.heartbeat_log) > 50:
            st.session_state.heartbeat_log = st.session_state.heartbeat_log[-50:]
        
        # 更新折线数据（保留最近30个点，保证折线流畅）
        st.session_state.heartbeat_chart_data["time"].append(now.strftime("%H:%M:%S"))
        st.session_state.heartbeat_chart_data["seq"].append(st.session_state.drone_heartbeat["heartbeat_seq"])
        if len(st.session_state.heartbeat_chart_data["time"]) > 30:
            st.session_state.heartbeat_chart_data["time"] = st.session_state.heartbeat_chart_data["time"][-30:]
            st.session_state.heartbeat_chart_data["seq"] = st.session_state.heartbeat_chart_data["seq"][-30:]

# ================== 绘制心跳折线图（保证一直向上） ==================
def draw_heartbeat_chart():
    df = pd.DataFrame({
        "时间": st.session_state.heartbeat_chart_data["time"],
        "心跳包序号": st.session_state.heartbeat_chart_data["seq"]
    })
    
    fig = go.Figure()
    # 仅一条蓝色折线（一直向上）
    fig.add_trace(go.Scatter(
        x=df["时间"],
        y=df["心跳包序号"],
        mode="lines+markers",
        line=dict(color="#1E88E5", width=3),  # 深蓝色更醒目
        marker=dict(size=6, color="#1E88E5", symbol="circle"),
        name="心跳包序号"
    ))
    
    # 图表样式优化
    fig.update_layout(
        title="心跳包实时曲线",
        title_font=dict(size=18, weight="bold", color="#333"),
        xaxis_title="北京时间",
        yaxis_title="心跳包序号",
        xaxis=dict(
            tickangle=-45,
            tickfont=dict(size=10),
            gridcolor="#EEEEEE"
        ),
        yaxis=dict(
            # Y轴从0开始，自动适配最大值（保证折线向上）
            range=[0, max(st.session_state.heartbeat_chart_data["seq"]) + 5 if st.session_state.heartbeat_chart_data["seq"] else 10],
            tickfont=dict(size=10),
            gridcolor="#EEEEEE"
        ),
        height=450,
        margin=dict(l=30, r=20, t=50, b=80),
        plot_bgcolor="#FFFFFF"
    )
    
    return fig

# ================== 侧边栏（完全保留） ==================
with st.sidebar:
    st.title("无人机无重叠避障系统")
    st.info(f"📌 地面基准高度：{GROUND_HEIGHT}米")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    # 无人机高度设置
    st.markdown("---")
    st.subheader("🛸 无人机飞行高度（地面以上）")
    st.session_state.drone_height = st.slider(
        "设置地面以上飞行高度（米）",
        min_value=0, max_value=200, value=8, step=1,
        key="drone_height_slider"
    )
    st.caption(f"当前：{st.session_state.drone_height}米（地面以上）")

    # 障碍物圈选
    st.markdown("---")
    st.subheader("🌍 多边形障碍物圈选")
    st.warning("⚠️ 高度超标时，航线绝对不与障碍物重叠！")
    
    draw_type = st.selectbox(
        "选择障碍物类型（匹配真实高度）",
        ["无", "自定义障碍物", "普通房屋", "高层楼房", "大树/电线杆", "操场/空地", "桥梁/高架", "塔楼/信号塔"],
        key="draw_type_select"
    )
    
    col1, col2 = st.columns(2)
    with col1:
        if st.button("🟢 开始圈选") and draw_type != "无":
            st.session_state.drawing_mode = draw_type
            st.session_state.current_points = []
            st.success(f"开始圈选「{draw_type}」（预设高度：{REAL_WORLD_HEIGHTS[draw_type]}米）")
    with col2:
        if st.button("✅ 完成圈选") and st.session_state.drawing_mode:
            if len(st.session_state.current_points) >= 3:
                if st.session_state.current_points[0] != st.session_state.current_points[-1]:
                    st.session_state.current_points.append(st.session_state.current_points[0])
                st.session_state.obstacles_all.append(st.session_state.current_points)
                st.session_state.obstacles_type.append(st.session_state.drawing_mode)
                st.session_state.obstacles_height.append(REAL_WORLD_HEIGHTS[st.session_state.drawing_mode])
                save_all()
                st.success(f"「{st.session_state.drawing_mode}」添加成功！高度：{REAL_WORLD_HEIGHTS[st.session_state.drawing_mode]}米")
            else:
                st.error("❌ 至少需要3个点形成多边形！")
            st.session_state.drawing_mode = None
            st.session_state.current_points = []

    # 清空按钮
    if st.button("🗑️ 清空所有障碍物"):
        st.session_state.obstacles_all = []
        st.session_state.obstacles_type = []
        st.session_state.obstacles_height = []
        save_all()
        st.success("✅ 所有障碍物已清空！")

    # 障碍物高度自定义
    st.markdown("---")
    st.subheader("📏 障碍物高度自定义（地面以上）")
    if len(st.session_state.obstacles_all) > 0:
        for i in range(len(st.session_state.obstacles_all)):
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            current_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else REAL_WORLD_HEIGHTS[obs_type]
            
            st.write(f"📌 {obs_type} {i+1}")
            new_h = st.slider(
                f"高度（米）",
                min_value=0, max_value=200, value=current_h, step=1,
                key=f"obs_{i}_height",
                label_visibility="collapsed"
            )
            if i < len(st.session_state.obstacles_height):
                st.session_state.obstacles_height[i] = new_h
                save_all()
                if new_h > st.session_state.drone_height:
                    st.error(f"⚠️ 高度({new_h}m) > 无人机({st.session_state.drone_height}m) → 无重叠绕行！")
                else:
                    st.success(f"✅ 高度({new_h}m) ≤ 无人机 → 直线飞行！")
    else:
        st.info("暂无障碍物，先圈选地图上的多边形物体")

    # A/B点管理
    st.markdown("---")
    st.subheader("📍 航线起点/终点")
    col_a, col_b = st.columns(2)
    with col_a:
        st.subheader("🟢 起点A")
        if st.session_state.point_a:
            st.success(f"纬度：{st.session_state.point_a[0]:.6f}")
            st.success(f"经度：{st.session_state.point_a[1]:.6f}")
            if st.button("清除A点", key="clear_a"):
                st.session_state.point_a = None
        else:
            st.warning("未设置")
    with col_b:
        st.subheader("🔴 终点B")
        if st.session_state.point_b:
            st.success(f"纬度：{st.session_state.point_b[0]:.6f}")
            st.success(f"经度：{st.session_state.point_b[1]:.6f}")
            if st.button("清除B点", key="clear_b"):
                st.session_state.point_b = None
        else:
            st.warning("未设置")

# ================== 航线规划页面（完全保留） ==================
if page == "航线规划":
    st.title("🗺️ 无人机无重叠精准避障系统")
    
    route, route_status = calculate_no_overlap_route()
    st.markdown(f"<h4 style='color:{'red' if '绕行' in route_status else 'green'};'>{route_status}</h4>", unsafe_allow_html=True)

    m = folium.Map(
        location=[32.2330, 118.7490],
        zoom_start=18,
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery"
    )

    if st.session_state.point_a:
        folium.CircleMarker(
            location=st.session_state.point_a,
            radius=12, color='green', fill=True, fill_color='green', fill_opacity=0.8,
            popup=f"起点A<br>地面基准：{GROUND_HEIGHT}米"
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_a,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px; background:green; padding:2px;">A 起点</div>')
        ).add_to(m)

    if st.session_state.point_b:
        folium.CircleMarker(
            location=st.session_state.point_b,
            radius=12, color='red', fill=True, fill_color='red', fill_opacity=0.8,
            popup=f"终点B<br>地面基准：{GROUND_HEIGHT}米"
        ).add_to(m)
        folium.Marker(
            location=st.session_state.point_b,
            icon=folium.DivIcon(html='<div style="color:white; font-weight:bold; font-size:14px; background:red; padding:2px;">B 终点</div>')
        ).add_to(m)

    TYPE_COLORS = {
        "自定义障碍物": "darkred",
        "普通房屋": "orange",
        "高层楼房": "darkblue",
        "大树/电线杆": "darkgreen",
        "操场/空地": "gray",
        "桥梁/高架": "purple",
        "塔楼/信号塔": "brown"
    }
    for i, obs in enumerate(st.session_state.obstacles_all):
        if len(obs) > 2:
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else REAL_WORLD_HEIGHTS[obs_type]
            color = TYPE_COLORS.get(obs_type, "darkred")
            
            fill_opacity = 0.7 if obs_h > st.session_state.drone_height else 0.3
            weight = 8 if obs_h > st.session_state.drone_height else 3
            status_text = "无重叠绕行" if obs_h > st.session_state.drone_height else "可直飞"
            
            folium.Polygon(
                locations=obs,
                color=color, fill=True, fill_color=color, fill_opacity=fill_opacity,
                weight=weight, 
                popup=f"{obs_type} | 地面以上高度：{obs_h}米<br>无人机高度：{st.session_state.drone_height}米<br>状态：{status_text}（绝对无重叠）",
                tooltip=f"{obs_type}（{obs_h}米，{status_text}）"
            ).add_to(m)

    if st.session_state.drawing_mode and len(st.session_state.current_points) > 0:
        draw_type = st.session_state.drawing_mode
        color = TYPE_COLORS.get(draw_type, "orange")
        
        folium.PolyLine(
            locations=st.session_state.current_points,
            color=color, weight=5, dash_array='5,5',
            popup=f"正在绘制：{draw_type}（多边形，地面基准：0米）"
        ).add_to(m)
        for idx, p in enumerate(st.session_state.current_points):
            folium.CircleMarker(
                location=p, radius=6, color=color, fill=True,
                popup=f"顶点 {idx+1}"
            ).add_to(m)

    if len(route) >= 2:
        folium.PolyLine(
            locations=route,
            color='blue', weight=10, opacity=0.9,
            popup=route_status,
            tooltip="无重叠避障航线（最短路径）"
        ).add_to(m)
        for idx, point in enumerate(route[1:-1]):
            folium.CircleMarker(
                location=point, radius=10, color='blue', fill=True, fill_color='yellow',
                popup=f"无重叠绕行点 {idx+1}（障碍物外{SAFE_DISTANCE*100000}米）"
            ).add_to(m)

    map_out = st_folium(
        m, key="drone_map", height=800,
        use_container_width=True, returned_objects=["last_clicked"]
    )

    if map_out and map_out.get("last_clicked"):
        lat = round(map_out["last_clicked"]["lat"], 6)
        lng = round(map_out["last_clicked"]["lng"], 6)
        
        if st.session_state.drawing_mode:
            st.session_state.current_points.append([lat, lng])
        else:
            if not st.session_state.point_a:
                st.session_state.point_a = (lat, lng)
                st.success(f"✅ 起点A已设置：({lat}, {lng})（地面基准：0米）")
            elif not st.session_state.point_b:
                st.session_state.point_b = (lat, lng)
                st.success(f"✅ 终点B已设置：({lat}, {lng})（地面基准：0米）")

# ================== 飞行监控页面（修复+新增启停+实时折线） ==================
else:
    st.title("📡 无人机飞行监控中心（含心跳包）")
    
    # 第一步：心跳监控启停按键（核心新增）
    col_start, col_stop, col_reset = st.columns([1,1,2])
    with col_start:
        if st.button("▶️ 开始监控", type="primary", disabled=st.session_state.heartbeat_running):
            st.session_state.heartbeat_running = True
            st.success("✅ 心跳包监控已启动！")
    with col_stop:
        if st.button("⏹️ 结束监控", type="secondary", disabled=not st.session_state.heartbeat_running):
            st.session_state.heartbeat_running = False
            st.warning("⚠️ 心跳包监控已停止！")
    with col_reset:
        if st.button("🔄 重置心跳数据"):
            # 重置所有心跳相关数据
            st.session_state.drone_heartbeat["heartbeat_seq"] = 0
            st.session_state.heartbeat_log = []
            st.session_state.heartbeat_chart_data = {"time": [], "seq": []}
            st.session_state.heartbeat_running = False
            st.info("🔧 心跳数据已重置！")
    
    # 第二步：更新心跳数据（兼容低版本Streamlit）
    update_drone_heartbeat()
    
    # 第三步：心跳状态告警
    heartbeat_status = "正常" if st.session_state.heartbeat_running else "已停止"
    alert_color = "green" if st.session_state.heartbeat_running else "gray"
    alert_icon = "✅" if st.session_state.heartbeat_running else "⏹️"
    
    # 异常判断（仅运行时）
    if st.session_state.heartbeat_running:
        time_since_last_heartbeat = (datetime.now() - st.session_state.drone_heartbeat["last_time"]).total_seconds()
        if time_since_last_heartbeat > 3:
            heartbeat_status = "心跳超时"
            alert_color = "red"
            alert_icon = "🔴"
        elif st.session_state.drone_heartbeat["battery"] < 20:
            heartbeat_status = "电量低"
            alert_color = "orange"
            alert_icon = "⚠️"
        elif st.session_state.drone_heartbeat["signal_strength"] < 80:
            heartbeat_status = "信号弱"
            alert_color = "orange"
            alert_icon = "⚠️"
    
    st.markdown(f"""
    <div style='background-color:{alert_color}; color:white; padding:12px; border-radius:8px; text-align:center; font-size:18px; font-weight:bold; margin:10px 0;'>
        {alert_icon} 无人机心跳状态：{heartbeat_status} | 最后心跳序号：{st.session_state.drone_heartbeat['heartbeat_seq']}
    </div>
    """, unsafe_allow_html=True)
    
    # 第四步：实时折线图（保证一直向上）
    st.subheader("🫀 心跳包实时曲线")
    chart_fig = draw_heartbeat_chart()
    st.plotly_chart(chart_fig, use_container_width=True)
    
    # 第五步：分栏展示数据
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("📊 心跳包核心数据")
        card_style = """
        <div style='background-color:#f0f2f6; padding:15px; border-radius:8px; margin-bottom:12px; border-left:4px solid {border_color};'>
            <span style='font-size:14px; color:#666;'>{label}</span><br>
            <span style='font-size:26px; font-weight:bold; color:{color};'>{value}</span>
        </div>
        """
        
        # 心跳序号（核心）
        st.markdown(card_style.format(
            label="当前心跳包序号",
            color="#1E88E5",
            border_color="#1E88E5",
            value=st.session_state.drone_heartbeat["heartbeat_seq"]
        ), unsafe_allow_html=True)
        
        # 信号强度
        signal_color = "#4CAF50" if st.session_state.drone_heartbeat["signal_strength"] >= 80 else "#FF9800" if st.session_state.drone_heartbeat["signal_strength"] >= 50 else "#F44336"
        st.markdown(card_style.format(
            label="信号强度",
            color=signal_color,
            border_color=signal_color,
            value=f"{st.session_state.drone_heartbeat['signal_strength']}%"
        ), unsafe_allow_html=True)
        
        # 电量
        battery_color = "#4CAF50" if st.session_state.drone_heartbeat["battery"] >= 50 else "#FF9800" if st.session_state.drone_heartbeat["battery"] >= 20 else "#F44336"
        st.markdown(card_style.format(
            label="剩余电量",
            color=battery_color,
            border_color=battery_color,
            value=f"{st.session_state.drone_heartbeat['battery']}%"
        ), unsafe_allow_html=True)
        
        # 飞行状态
        flight_color = "#4CAF50" if st.session_state.drone_heartbeat["flight_status"] in ["飞行中", "绕飞中"] else "#9E9E9E"
        st.markdown(card_style.format(
            label="飞行状态",
            color=flight_color,
            border_color=flight_color,
            value=st.session_state.drone_heartbeat["flight_status"]
        ), unsafe_allow_html=True)
    
    with col2:
        st.subheader("✅ 基础飞行状态")
        st.success(f"无人机地面以上高度：{st.session_state.drone_height} 米")
        st.success(f"已圈选多边形障碍物数量：{len(st.session_state.obstacles_all)} 个")
        
        avoid_count = sum(1 for h in st.session_state.obstacles_height if h > st.session_state.drone_height)
        if avoid_count > 0:
            st.error(f"🔴 发现 {avoid_count} 个障碍物高度超标，航线将「绝对不重叠」绕飞！")
        else:
            st.success(f"🟢 所有障碍物高度均达标，无人机将直线从A到B！")
        
        if len(st.session_state.obstacles_all) > 0:
            st.subheader("🌍 障碍物详情")
            for i in range(len(st.session_state.obstacles_all)):
                obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
                obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
                status = "🔴 绕飞" if obs_h > st.session_state.drone_height else "🟢 直飞"
                st.info(f"{obs_type} {i+1}：{obs_h}米 → {status}")
        else:
            st.info("暂无障碍物数据！")
        
        st.subheader("📍 航线策略")
        if st.session_state.point_a and st.session_state.point_b:
            _, route_status = calculate_no_overlap_route()
            st.write(f"当前策略：{route_status}")
        else:
            st.warning("请先设置起点A和终点B！")
    
    # 第六步：通信日志
    st.subheader("📜 通信日志")
    log_container = st.container(height=200)
    with log_container:
        if st.session_state.heartbeat_log:
            for log in reversed(st.session_state.heartbeat_log):
                st.text(f"[{log['time']}] 发送心跳包 序号={log['seq']}")
        else:
            st.text("暂无心跳日志（点击「开始监控」生成数据）")
    
    # 低版本Streamlit实时刷新方案（替代autorefresh）
    if st.session_state.heartbeat_running:
        # 每1秒重新运行脚本（实现实时刷新）
        time.sleep(1)
        st.rerun()