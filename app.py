import streamlit as st
import pandas as pd
import time
import math
import random
from datetime import datetime

# ================== 坐标系转换（GCJ-02 ↔ WGS-84）==================
def gcj02_to_wgs84(lat, lng):
    a = 6378245.0
    ee = 0.00669342162296594323

    def transform_lat(x, y):
        ret = -100.0 + 2.0*x + 3.0*y + 0.2*y*y + 0.1*x*y + 0.2*math.sqrt(abs(x))
        ret += (20.0*math.sin(6.0*x*math.pi) + 20.0*math.sin(2.0*x*math.pi)) * 2.0/3.0
        ret += (20.0*math.sin(y*math.pi) + 40.0*math.sin(y/3.0*math.pi)) * 2.0/3.0
        ret += (160.0*math.sin(y/12.0*math.pi) + 320*math.sin(y*math.pi/30.0)) * 2.0/3.0
        return ret

    def transform_lng(x, y):
        ret = 300.0 + x + 2.0*y + 0.1*x*x + 0.1*x*y + 0.1*math.sqrt(abs(x))
        ret += (20.0*math.sin(6.0*x*math.pi) + 20.0*math.sin(2.0*x*math.pi)) * 2.0/3.0
        ret += (20.0*math.sin(x*math.pi) + 40.0*math.sin(x/3.0*math.pi)) * 2.0/3.0
        ret += (150.0*math.sin(x/12.0*math.pi) + 300.0*math.sin(x/30.0*math.pi)) * 2.0/3.0
        return ret

    dlat = transform_lat(lng-105.0, lat-35.0)
    dlng = transform_lng(lng-105.0, lat-35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat*180.0) / ((a*(1-ee))/(magic*sqrtmagic)*math.pi)
    dlng = (dlng*180.0) / (a/sqrtmagic*math.cos(radlat)*math.pi)
    return lat-dlat, lng-dlng

# ================== 状态初始化 ==================
for k in ['obstacles', 'click_lat', 'click_lng', 'a_point', 'b_point', 'coord_sys',
          'heartbeat_data','last_heartbeat_time','is_connected','heartbeat_sequence','simulation_on']:
    if k not in st.session_state:
        if k == 'obstacles': st.session_state[k] = []
        elif k in ['click_lat','click_lng']: st.session_state[k] = None
        elif k == 'a_point': st.session_state[k] = (32.2322, 118.7490, "GCJ-02")
        elif k == 'b_point': st.session_state[k] = (32.2343, 118.7490, "GCJ-02")
        elif k == 'coord_sys': st.session_state[k] = "GCJ-02"
        elif k == 'heartbeat_data': st.session_state[k] = pd.DataFrame(columns=['序号','时间'])
        elif k == 'last_heartbeat_time': st.session_state[k] = datetime.now()
        elif k == 'is_connected': st.session_state[k] = True
        elif k == 'heartbeat_sequence': st.session_state[k] = 0
        elif k == 'simulation_on': st.session_state[k] = False

# ================== 心跳 ==================
def add_heartbeat():
    st.session_state.heartbeat_sequence +=1
    new_row = pd.DataFrame([{'序号':st.session_state.heartbeat_sequence,'时间':datetime.now()}])
    st.session_state.heartbeat_data = pd.concat([st.session_state.heartbeat_data, new_row], ignore_index=True)
    st.session_state.last_heartbeat_time = datetime.now()
    st.session_state.is_connected = True

def check_connection():
    if (datetime.now()-st.session_state.last_heartbeat_time).total_seconds()>3:
        st.session_state.is_connected=False

# ================== 页面 ==================
st.set_page_config(page_title="无人机监控", layout="wide")
st.sidebar.title("📡 无人机导航")
page = st.sidebar.radio("页面", ["🗺️ 航线规划", "📶 飞行监控"])

# ================== 航线规划 ==================
if page == "🗺️ 航线规划":
    st.title("🗺️ 航线规划 & 障碍物设置")
    a_lat_raw, a_lng_raw, a_sys = st.session_state.a_point
    b_lat_raw, b_lng_raw, b_sys = st.session_state.b_point

    # 左侧面板
    with st.sidebar:
        st.markdown("---")
        st.subheader("🧱 障碍物")
        st.info("1. 点击地图\n2. 点确认添加")
        if st.button("✅ 确认添加障碍物"):
            if st.session_state.click_lat and st.session_state.click_lng:
                st.session_state.obstacles.append((
                    st.session_state.click_lat,
                    st.session_state.click_lng,
                    random.randint(20,60)
                ))
                st.success("添加成功")
                st.session_state.click_lat=None
                st.session_state.click_lng=None
            else:
                st.warning("先点地图")
        if st.button("🔄 清空障碍物"):
            st.session_state.obstacles=[]
            st.success("已清空")

        st.markdown("---")
        st.subheader("🌐 坐标系")
        coord_opt = ["GCJ-02","WGS-84"]
        sel = st.radio("选择", coord_opt, index=coord_opt.index(st.session_state.coord_sys))
        if st.button("✅ 应用到A/B点"):
            st.session_state.coord_sys = sel
            st.session_state.a_point = (a_lat_raw, a_lng_raw, sel)
            st.session_state.b_point = (b_lat_raw, b_lng_raw, sel)
            st.success("已应用")

    # 坐标转WGS用于地图
    def to_wgs(lat,lng,sys):
        return gcj02_to_wgs84(lat,lng) if sys=="GCJ-02" else (lat,lng)
    a_lat,a_lng = to_wgs(a_lat_raw,a_lng_raw,a_sys)
    b_lat,b_lng = to_wgs(b_lat_raw,b_lng_raw,b_sys)

    # 障碍物JS
    obs_js = ""
    for i,(lat,lng,h) in enumerate(st.session_state.obstacles):
        obs_js += f"""
        L.circle([{lat},{lng}],{{color:'orange',fillColor:'#ff2',fillOpacity:0.7,radius:15}}).addTo(map);
        L.marker([{lat},{lng}],{{icon:L.divIcon({{html:'{i+1}',className:'obs',iconSize:[22,22]}})}}).addTo(map);
        """
    temp_js = f"L.marker([{st.session_state.click_lat},{st.session_state.click_lng}]).addTo(map);" if st.session_state.click_lat else ""

    # 地图HTML（修复CSS/JS/瓦片/高度）
    map_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>
            #map {{width:100%;height:600px;}}
            .obs {{background:orange;color:white;font-weight:bold;border-radius:50%;text-align:center;line-height:22px;}}
        </style>
    </head>
    <body>
        <div id="map"></div>
        <script>
            const map = L.map('map').setView([{(a_lat+b_lat)/2},{(a_lng+b_lng)/2}], 18);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{{
                attribution:'&copy; OpenStreetMap'
            }}).addTo(map);
            // A/B
            L.marker([{a_lat},{a_lng}]);.addTo(map).bindPopup('A起点');
            L.marker([{b_lat},{b_lng}]);.addTo(map).bindPopup('B终点');
            L.polyline([[{a_lat},{a_lng}],[{b_lat},{b_lng}]],{{color:'blue',weight:5}}).addTo(map);
            // 障碍物
            {obs_js}
            {temp_js}
            // 点击
            map.on('click', e => {{
                window.parent.postMessage({{
                    type:'mapClick', lat:e.latlng.lat, lng:e.latlng.lng
                }}, '*');
            }});
        </script>
    </body>
    </html>
    """

    # 渲染地图（关键修复：height/width 给足）
    from streamlit.components.v1 import html
    html(map_html, height=600, width=1100)

    # 接收点击
    try:
        msg = st.runtime.scriptrunner.add_script_run_ctx(lambda: None)
        # 兼容新版Streamlit点击接收
        import jsonschema
        from streamlit import components
        msg = components.v1.get_component_message("mapClick")
        if msg:
            st.session_state.click_lat = msg["lat"]
            st.session_state.click_lng = msg["lng"]
    except:
        pass

    # A/B点输入
    st.markdown("---")
    c1,c2 = st.columns(2)
    with c1:
        st.subheader("📍 A点")
        a1=st.number_input("A纬度",a_lat_raw,format="%.6f")
        a2=st.number_input("A经度",a_lng_raw,format="%.6f")
        if st.button("✅ 设置A点"):
            st.session_state.a_point=(a1,a2,st.session_state.coord_sys)
    with c2:
        st.subheader("📍 B点")
        b1=st.number_input("B纬度",b_lat_raw,format="%.6f")
        b2=st.number_input("B经度",b_lng_raw,format="%.6f")
        if st.button("✅ 设置B点"):
            st.session_state.b_point=(b1,b2,st.session_state.coord_sys)

    # 障碍物列表
    st.markdown("---")
    st.subheader(f"障碍物：{len(st.session_state.obstacles)}")
    if st.session_state.obstacles:
        with st.expander("查看"):
            for i,(lat,lng,h) in enumerate(st.session_state.obstacles):
                st.write(f"{i+1} ｜ {lat:.6f}, {lng:.6f} ｜ {h}m")

# ================== 飞行监控 ==================
else:
    st.title("📶 飞行心跳监控")
    c1,c2=st.columns(2)
    with c1:
        if st.button("▶️ 开始模拟"):
            st.session_state.simulation_on=True
    with c2:
        if st.button("⏸️ 停止"):
            st.session_state.simulation_on=False

    if st.session_state.simulation_on:
        add_heartbeat()
        check_connection()
        time.sleep(1)
        st.rerun()

    if st.session_state.is_connected:
        st.success("✅ 在线")
    else:
        st.error("🚨 失联")

    if not st.session_state.heartbeat_data.empty:
        last=st.session_state.heartbeat_data.iloc[-1]
        st.info(f"最新心跳 {last['序号']}｜{last['时间'].strftime('%H:%M:%S')}")

    st.subheader("📈 心跳趋势")
    df=st.session_state.heartbeat_data.tail(50)
    if not df.empty:
        df['时间']=pd.to_datetime(df['时间'])
        st.line_chart(df.set_index('时间')['序号'])

    if st.button("🗑️ 清空心跳"):
        st.session_state.heartbeat_data=pd.DataFrame(columns=['序号','时间'])
        st.session_state.heartbeat_sequence=0