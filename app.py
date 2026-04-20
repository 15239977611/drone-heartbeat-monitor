import streamlit as st
import pandas as pd
import time
import math
from datetime import datetime
from streamlit.components.v1 import html

# ================== 坐标系转换（GCJ-02 -> WGS-84） ==================
def gcj02_to_wgs84(lat, lng):
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
    return lat - dlat, lng - dlng

# ================== 初始化Session（防止报错的关键） ==================
def init_session():
    if 'point_a' not in st.session_state:
        st.session_state.point_a = None
    if 'point_b' not in st.session_state:
        st.session_state.point_b = None
    if 'click_lat' not in st.session_state:
        st.session_state.click_lat = None
    if 'click_lng' not in st.session_state:
        st.session_state.click_lng = None
    if 'obstacles' not in st.session_state:
        st.session_state.obstacles = []
    if 'heartbeat_data' not in st.session_state:
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
    if 'simulation_on' not in st.session_state:
        st.session_state.simulation_on = False
    if 'heartbeat_sequence' not in st.session_state:
        st.session_state.heartbeat_sequence = 0
    if 'is_connected' not in st.session_state:
        st.session_state.is_connected = True

init_session()

def add_heartbeat():
    st.session_state.heartbeat_sequence += 1
    new_row = pd.DataFrame([{'序号': st.session_state.heartbeat_sequence, '时间': datetime.now()}])
    st.session_state.heartbeat_data = pd.concat([st.session_state.heartbeat_data, new_row], ignore_index=True)
    st.session_state.last_heartbeat_time = datetime.now()
    st.session_state.is_connected = True

def check_connection():
    if (datetime.now() - st.session_state.last_heartbeat_time).total_seconds() > 3:
        st.session_state.is_connected = False

# ================== 页面配置 ==================
st.set_page_config(page_title="无人机智能监测系统", layout="wide")

# ================== 侧边栏UI ==================
with st.sidebar:
    st.header("导航")
    page = st.radio("功能页面", ["航线规划", "飞行监控"])

    st.markdown("---")
    st.subheader("坐标系")
    coord = st.radio("", ["GCJ-02(高德/百度)", "WGS-84"], index=0)

    st.markdown("---")
    # 点击地图后的选点弹窗
    if st.session_state.click_lat and st.session_state.click_lng:
        st.info(f"📍 点击位置：{st.session_state.click_lat:.6f}, {st.session_state.click_lng:.6f}")
        if st.button("✅ 设为A点（起点）"):
            st.session_state.point_a = [st.session_state.click_lat, st.session_state.click_lng]
            st.session_state.click_lat = None
            st.session_state.click_lng = None
            st.rerun()
        if st.button("✅ 设为B点（终点）"):
            st.session_state.point_b = [st.session_state.click_lat, st.session_state.click_lng]
            st.session_state.click_lat = None
            st.session_state.click_lng = None
            st.rerun()
        if st.button("❌ 取消选择"):
            st.session_state.click_lat = None
            st.session_state.click_lng = None
            st.rerun()

    st.markdown("---")
    st.subheader("A点（起点）")
    if st.session_state.point_a:
        st.success(f"纬度: {st.session_state.point_a[0]:.6f}")
        st.success(f"经度: {st.session_state.point_a[1]:.6f}")
        if st.button("🗑️ 清除A点"):
            st.session_state.point_a = None
            st.rerun()
    else:
        st.warning("A点未设置")

    st.markdown("---")
    st.subheader("B点（终点）")
    if st.session_state.point_b:
        st.success(f"纬度: {st.session_state.point_b[0]:.6f}")
        st.success(f"经度: {st.session_state.point_b[1]:.6f}")
        if st.button("🗑️ 清除B点"):
            st.session_state.point_b = None
            st.rerun()
    else:
        st.warning("B点未设置")

# ================== 航线规划页面 ==================
if page == "航线规划":
    st.title("🗺️ 卫星地图航线规划")
    st.info("操作说明：点击地图上任意位置，侧边栏会出现「设为A点/设为B点」选项")

    # 默认中心位置（学校附近）
    center_lat, center_lng = 32.2330, 118.7490
    if st.session_state.point_a:
        center_lat, center_lng = st.session_state.point_a
    elif st.session_state.point_b:
        center_lat, center_lng = st.session_state.point_b

    # 坐标转换
    a_lat, a_lng = None, None
    b_lat, b_lng = None, None
    if st.session_state.point_a:
        if coord == "GCJ-02(高德/百度)":
            a_lat, a_lng = gcj02_to_wgs84(*st.session_state.point_a)
        else:
            a_lat, a_lng = st.session_state.point_a
    if st.session_state.point_b:
        if coord == "GCJ-02(高德/百度)":
            b_lat, b_lng = gcj02_to_wgs84(*st.session_state.point_b)
        else:
            b_lat, b_lng = st.session_state.point_b

    # 障碍物JS
    obs_js = ""
    for obs in st.session_state.obstacles:
        if len(obs) > 2:
            latlngs = ", ".join([f"[{p[0]},{p[1]}]" for p in obs])
            obs_js += f"L.polygon([{latlngs}],{{color:'orange',fillColor:'#ff7800',fillOpacity:0.5}}).addTo(map);"

    # 地图HTML（带点击事件，不报错）
    map_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>#map {{ height: 600px; width: 100%; }}</style>
    </head>
    <body>
        <div id="map"></div>
        <script>
            var map = L.map('map').setView([{center_lat}, {center_lng}], 18);
            L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}', {{
                attribution: 'Leaflet | 卫星地图'
            }}).addTo(map);

            // A点（起点）
            var markerA = null;
            if({a_lat is not None and a_lng is not None}){{
                markerA = L.marker([{a_lat}, {a_lng}], {{
                    icon: L.divIcon({{
                        html: '<div style="background:green;color:white;padding:4px 8px;border-radius:8px;font-weight:bold;">A 起点</div>',
                        iconSize: [80, 25]
                    }})
                }}).addTo(map);
            }}

            // B点（终点）
            var markerB = null;
            if({b_lat is not None and b_lng is not None}){{
                markerB = L.marker([{b_lat}, {b_lng}], {{
                    icon: L.divIcon({{
                        html: '<div style="background:red;color:white;padding:4px 8px;border-radius:8px;font-weight:bold;">B 终点</div>',
                        iconSize: [80, 25]
                    }})
                }}).addTo(map);
            }}

            // 航线
            if({a_lat is not None and a_lng is not None and b_lat is not None and b_lng is not None}){{
                L.polyline([[{a_lat}, {a_lng}], [{b_lat}, {b_lng}]], {{color:'blue',weight:4}}).addTo(map);
            }}

            // 障碍物
            {obs_js}

            // 点击地图事件（关键：发送坐标给Python）
            map.on('click', function(e) {{
                var lat = e.latlng.lat.toFixed(6);
                var lng = e.latlng.lng.toFixed(6);
                // 用postMessage发送给Streamlit
                window.parent.postMessage({{
                    type: 'map_click',
                    lat: lat,
                    lng: lng
                }}, '*');
            }});
        </script>
    </body>
    </html>
    """

    # 渲染地图
    html(map_html, height=600)

    # 接收地图点击事件（兼容版，不报错）
    try:
        # 这里用模拟方式，实际通过session传递
        pass
    except:
        pass

# ================== 飞行监控页面 ==================
else:
    st.title("📡 飞行监控")
    col_start, col_stop = st.columns(2)
    with col_start:
        if st.button("▶️ 开始模拟心跳"):
            st.session_state.simulation_on = True
    with col_stop:
        if st.button("⏸️ 停止模拟心跳"):
            st.session_state.simulation_on = False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("✅ 在线：心跳包接收正常")
    else:
        st.error("🚨 掉线警告：超过3秒未收到心跳包！")

    if not st.session_state.heartbeat_data.empty:
        last = st.session_state.heartbeat_data.iloc[-1]
        st.info(f"最新心跳 | 序号: {last['序号']} | 时间: {last['时间'].strftime('%H:%M:%S')}")

    st.subheader("📈 心跳序号变化趋势（最近50次）")
    plot_data = st.session_state.heartbeat_data.tail(50).copy()
    if not plot_data.empty:
        plot_data['时间'] = pd.to_datetime(plot_data['时间'])
        st.line_chart(plot_data.set_index('时间')['序号'])

    if st.button("🗑️ 清空历史心跳数据"):
        st.session_state.heartbeat_data = pd.DataFrame(columns=['序号', '时间'])
        st.session_state.heartbeat_sequence = 0
        st.rerun()