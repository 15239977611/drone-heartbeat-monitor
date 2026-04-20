import streamlit as st
import pandas as pd
import time
import random
import math
from datetime import datetime
import pydeck as pdk
import numpy as np

# ================== 坐标系转换（GCJ-02 <-> WGS-84） ==================
# 简化版转换算法（精度足够用于演示）
def gcj02_to_wgs84(lat, lng):
    """GCJ-02 转 WGS-84"""
    a = 6378245.0
    ee = 0.00669342162296594323
    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
        return ret
    def transform_lng(x, y):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
        return ret
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    wgs_lat = lat - dlat
    wgs_lng = lng - dlng
    return wgs_lat, wgs_lng

def wgs84_to_gcj02(lat, lng):
    """WGS-84 转 GCJ-02（简单反向近似）"""
    # 实际应用中可以忽略，这里只做占位
    return lat, lng

# ================== 障碍物生成 ==================
def generate_obstacles(a_lat, a_lng, b_lat, b_lng, num=5):
    """在AB连线两侧随机生成障碍物点（模拟建筑物/树木）"""
    obstacles = []
    for i in range(num):
        # 插值参数 t
        t = (i + 1) / (num + 1)
        base_lat = a_lat + (b_lat - a_lat) * t
        base_lng = a_lng + (b_lng - a_lng) * t
        # 垂直方向偏移约 0.0002度 ≈ 22米
        offset_lat = (random.random() - 0.5) * 0.0004
        offset_lng = (random.random() - 0.5) * 0.0004
        obs_lat = base_lat + offset_lat
        obs_lng = base_lng + offset_lng
        obstacles.append([obs_lat, obs_lng, random.randint(20, 50)])  # 高度20-50米
    return obstacles

# ================== 心跳模拟（全局数据存储） ==================
if 'heartbeat_data' not in st.session_state:
    st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
if 'last_heartbeat_time' not in st.session_state:
    st.session_state.last_heartbeat_time = datetime.now()
if 'is_connected' not in st.session_state:
    st.session_state.is_connected = True
if 'heartbeat_sequence' not in st.session_state:
    st.session_state.heartbeat_sequence = 0

def add_heartbeat():
    st.session_state.heartbeat_sequence += 1
    new_row = pd.DataFrame([{
        '序号': st.session_state.heartbeat_sequence,
        '时间': datetime.now()
    }])
    st.session_state.heartbeat_data = pd.concat([st.session_state.heartbeat_data, new_row], ignore_index=True)
    st.session_state.last_heartbeat_time = datetime.now()
    st.session_state.is_connected = True

def check_connection():
    now = datetime.now()
    if (now - st.session_state.last_heartbeat_time).total_seconds() > 3:
        st.session_state.is_connected = False

# ================== 页面配置 ==================
st.set_page_config(page_title="无人机智能监测系统", layout="wide")

# 侧边栏导航
pg = st.navigation([
    st.Page("航线规划", title="航线规划", icon="🗺️"),
    st.Page("飞行监控", title="飞行监控", icon="📡")
])

# 为了让两个页面共享 session_state，我们定义两个函数作为页面内容
def page_route_planning():
    st.title("🗺️ 航线规划")
    st.markdown("设置起飞点A（校园内）和降落点B（校园外），自动生成障碍物。")

    # 默认坐标：南京科技职业学院附近（GCJ-02）
    # A点：校园内（约 32.2322, 118.749）
    # B点：校外（约 32.2343, 118.749）
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("起点 A（校园内）")
        coord_sys = st.radio("坐标系", ["GCJ-02 (高德/百度)", "WGS-84"], key="coord_sys")
        a_lat = st.number_input("纬度 A", value=32.2322, format="%.6f", key="a_lat")
        a_lng = st.number_input("经度 A", value=118.7490, format="%.6f", key="a_lng")
        if st.button("设置 A 点", key="set_a"):
            st.session_state.a_point = (a_lat, a_lng, coord_sys)
            st.success("A点已保存")
    with col2:
        st.subheader("终点 B（校外）")
        b_lat = st.number_input("纬度 B", value=32.2343, format="%.6f", key="b_lat")
        b_lng = st.number_input("经度 B", value=118.7490, format="%.6f", key="b_lng")
        if st.button("设置 B 点", key="set_b"):
            st.session_state.b_point = (b_lat, b_lng, coord_sys)
            st.success("B点已保存")

    flight_height = st.slider("飞行高度 (米)", 10, 200, 50)

    # 显示当前已设状态
    st.markdown("---")
    st.subheader("系统状态")
    col3, col4 = st.columns(2)
    col3.metric("A点已设", "✅" if 'a_point' in st.session_state else "❌")
    col4.metric("B点已设", "✅" if 'b_point' in st.session_state else "❌")

    # 地图显示
    if 'a_point' in st.session_state and 'b_point' in st.session_state:
        a_lat, a_lng, a_sys = st.session_state.a_point
        b_lat, b_lng, b_sys = st.session_state.b_point

        # 将坐标统一转换为 WGS-84 用于地图显示
        if a_sys == "GCJ-02 (高德/百度)":
            a_lat_wgs, a_lng_wgs = gcj02_to_wgs84(a_lat, a_lng)
        else:
            a_lat_wgs, a_lng_wgs = a_lat, a_lng
        if b_sys == "GCJ-02 (高德/百度)":
            b_lat_wgs, b_lng_wgs = gcj02_to_wgs84(b_lat, b_lng)
        else:
            b_lat_wgs, b_lng_wgs = b_lat, b_lng

        # 生成障碍物（基于WGS-84坐标）
        obstacles = generate_obstacles(a_lat_wgs, a_lng_wgs, b_lat_wgs, b_lng_wgs, num=5)

        # 创建 pydeck 3D 地图
        st.subheader("3D 航线地图（可拖拽/旋转）")

        # 起点和终点图层（圆形标记）
        start_layer = pdk.Layer(
            "ScatterplotLayer",
            data=[{"lat": a_lat_wgs, "lng": a_lng_wgs, "color": [0, 255, 0], "size": 200}],
            get_position='[lng, lat]',
            get_color='color',
            get_radius='size',
            pickable=True,
        )
        end_layer = pdk.Layer(
            "ScatterplotLayer",
            data=[{"lat": b_lat_wgs, "lng": b_lng_wgs, "color": [255, 0, 0], "size": 200}],
            get_position='[lng, lat]',
            get_color='color',
            get_radius='size',
        )
        # 航线（线图层）
        line_layer = pdk.Layer(
            "LineLayer",
            data=[{"path": [[a_lng_wgs, a_lat_wgs], [b_lng_wgs, b_lat_wgs]]}],
            get_path='path',
            get_color=[100, 100, 255],
            get_width=10,
            pickable=True,
        )
        # 障碍物（用柱状图层 ColumnLayer 表示高度）
        obs_data = [{"lat": o[0], "lng": o[1], "height": o[2]} for o in obstacles]
        obstacle_layer = pdk.Layer(
            "ColumnLayer",
            data=obs_data,
            get_position='[lng, lat]',
            get_elevation='height',
            elevation_scale=1,
            radius=10,
            get_fill_color=[255, 165, 0, 180],
            pickable=True,
            auto_highlight=True,
        )

        view_state = pdk.ViewState(
            latitude=(a_lat_wgs + b_lat_wgs) / 2,
            longitude=(a_lng_wgs + b_lng_wgs) / 2,
            zoom=16,
            pitch=50,
            bearing=0,
        )

        r = pdk.Deck(
            layers=[start_layer, end_layer, line_layer, obstacle_layer],
            initial_view_state=view_state,
            tooltip={"text": "{name}"},
        )
        st.pydeck_chart(r)

        # 显示障碍物列表
        with st.expander("查看障碍物位置（WGS-84）"):
            for i, obs in enumerate(obstacles):
                st.write(f"障碍物 {i+1}: 纬度 {obs[0]:.6f}, 经度 {obs[1]:.6f}, 高度 {obs[2]}米")
    else:
        st.info("请先设置起点 A 和终点 B，地图将自动显示航线及障碍物。")

def page_flight_monitor():
    st.title("📡 飞行监控")
    st.markdown("实时心跳包监测（每秒一次），3秒未收到则报警。")

    # 自动添加心跳（使用 st.empty + 自动刷新）
    # 为了让心跳持续，我们用一个 checkbox 控制模拟开关
    if 'simulation_on' not in st.session_state:
        st.session_state.simulation_on = True

    col_start, col_stop = st.columns(2)
    with col_start:
        if st.button("▶️ 开始模拟心跳"):
            st.session_state.simulation_on = True
    with col_stop:
        if st.button("⏸️ 停止模拟心跳"):
            st.session_state.simulation_on = False

    # 自动刷新占位符
    placeholder = st.empty()

    # 每1秒执行一次心跳添加
    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        # 刷新页面（相当于每1秒自动重跑脚本）
        time.sleep(1)
        st.rerun()

    # 显示连接状态
    if st.session_state.is_connected:
        st.success("✅ 在线：心跳包接收正常")
    else:
        st.error("🚨 掉线警告：超过3秒未收到心跳包！")

    # 显示最新心跳
    if not st.session_state.heartbeat_data.empty:
        last = st.session_state.heartbeat_data.iloc[-1]
        st.info(f"最新心跳 | 序号: {last['序号']} | 时间: {last['时间'].strftime('%H:%M:%S')}")

    # 折线图
    st.subheader("心跳序号变化趋势（最近50次）")
    plot_data = st.session_state.heartbeat_data.tail(50).copy()
    if not plot_data.empty:
        plot_data['时间'] = pd.to_datetime(plot_data['时间'])
        st.line_chart(plot_data.set_index('时间')['序号'])

    # 可选：清除历史数据按钮
    if st.button("清空历史心跳数据"):
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
        st.session_state.heartbeat_sequence = 0
        st.rerun()

# ================== 主入口 ==================
# 由于 st.navigation 需要传入页面对象，但页面内容是函数，这里使用字典方式模拟
# 更简单：直接使用 radio 选择页面，避免 st.navigation 复杂语法

# 重写主逻辑：用侧边栏选择
st.sidebar.title("导航")
page = st.sidebar.radio("功能页面", ["航线规划", "飞行监控"])

if page == "航线规划":
    page_route_planning()
else:
    page_flight_monitor()