import streamlit as st
import math
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
from shapely.affinity import scale

# 辅助函数：计算两点间距离（经纬度坐标系）
def calculate_distance(point1, point2):
    """
    计算两个经纬度点之间的距离（单位：度）
    后续可通过 1度≈111公里 转换为实际距离
    """
    return math.hypot(point1[0] - point2[0], point1[1] - point2[1])

# 辅助函数：获取动态安全距离（经纬度单位）
def get_safe_distance():
    """
    转换无人机安全半径（米）为经纬度单位
    近似：1米 ≈ 9.009e-6 度（纬度），这里做简化处理
    """
    safety_radius_m = st.session_state.get("drone_safety_radius", 50)  # 默认50米
    return safety_radius_m * 9.009e-6  # 米转经纬度度数

def calculate_shortest_no_overlap_route():
    """
    修正版：实体多边形障碍物完整外围绕行（非切角）
    核心逻辑：
    1. 实体障碍物视为实心，无人机必须绕完整外轮廓（安全半径外）
    2. 找起点到障碍物轮廓的最近点 → 沿轮廓所有顶点 → 轮廓到终点的最近点
    3. 保证路径最短且不穿透实体
    4. 支持多障碍物依次绕行，自动选择顺时针/逆时针最短绕行方向
    """
    # 初始化返回值
    final_route = []
    status = ""
    
    try:
        # 优先使用转换后的坐标（无则用原始坐标）
        A = st.session_state.transformed_points.get("point_a") or st.session_state.get("point_a")
        B = st.session_state.transformed_points.get("point_b") or st.session_state.get("point_b")
        
        # 校验起点终点
        if not A or not B:
            return [], "❌ 未设置起点A/终点B（地面基准：0米）"
        
        # 获取无人机参数
        drone_h = st.session_state.get("drone_height", 0)
        SAFE_DISTANCE = get_safe_distance()  # 动态安全距离（经纬度单位）
        final_route = [A]
        avoid_obstacles = []
        
        # 优先使用转换后的障碍物坐标
        obstacles = st.session_state.transformed_points.get("obstacles") or st.session_state.get("obstacles_all", [])
        
        # 过滤需要避开的实体障碍物（高度>无人机高度，且是有效多边形）
        solid_obstacles = []
        for i, obs_coords in enumerate(obstacles):
            # 过滤无效多边形（至少3个顶点）
            if len(obs_coords) < 3:
                continue
            
            # 获取障碍物属性
            obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.get("obstacles_height", [])) else 50
            obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.get("obstacles_type", [])) else "自定义障碍物"
            
            # 仅处理高度超过无人机飞行高度的障碍物
            if obs_h <= drone_h:
                continue
            
            avoid_obstacles.append(obs_type)
            
            # 1. 构建实体障碍物原始多边形
            obs_poly = Polygon(obs_coords)
            if not obs_poly.is_valid:
                continue  # 跳过无效多边形
            
            # 2. 按安全半径外扩（保证无人机在实体外安全距离）
            # 外扩系数：基于安全半径（米转比例），以质心为原点外扩
            scale_factor = 1.0 + (st.session_state.get("drone_safety_radius", 50) / 1000)
            obs_poly_safe = scale(obs_poly, xfact=scale_factor, yfact=scale_factor, origin='centroid')
            
            # 3. 提取实体障碍物的完整外轮廓顶点（移除闭合重复点）
            contour_vertices = list(obs_poly_safe.exterior.coords)
            if contour_vertices and contour_vertices[0] == contour_vertices[-1]:
                contour_vertices = contour_vertices[:-1]  # 移除重复的闭合点
            
            # 4. 存储有效障碍物信息
            solid_obstacles.append({
                "poly": obs_poly_safe,       # 安全距离外的实体多边形
                "contour": contour_vertices, # 实体外轮廓顶点列表
                "type": obs_type,
                "height": obs_h,
                "centroid": obs_poly_safe.centroid,  # 实体中心（用于判断绕行方向）
                "original_coords": obs_coords        # 原始坐标（备用）
            })
        
        # 无需要避开的实体障碍物：直接直线飞行
        if not solid_obstacles:
            final_route = [A, B]
            total_distance = calculate_distance(A, B) * 111
            status = (
                f"🟢 最短直线飞行！\n"
                f"无人机高度({drone_h}m) ≥ 所有实体障碍物高度，直接从A到B\n"
                f"安全半径：{st.session_state.get('drone_safety_radius', 50)}米 | "
                f"总路径长度≈{total_distance:.3f}公里"
            )
            return final_route, status
        
        # ========== 核心：多实体障碍物完整绕行逻辑 ==========
        current_point = A  # 当前位置，初始为起点A
        target_point = B   # 最终目标点B
        
        # 遍历所有需要绕行的实体障碍物
        for obs_idx, obs in enumerate(solid_obstacles):
            obs_poly = obs["poly"]
            obs_contour = obs["contour"]
            obs_centroid = obs["centroid"]
            
            # 1. 检查当前点→终点的直线是否穿透当前障碍物
            direct_line = LineString([current_point, target_point])
            if not direct_line.intersects(obs_poly):
                continue  # 不穿透当前障碍物，跳过
            
            # 2. 找「当前点→实体轮廓」的最近点（绕行起点）
            current_to_obs = nearest_points(Point(current_point), obs_poly.boundary)
            start_avoid_point = (current_to_obs[1].x, current_to_obs[1].y)
            
            # 3. 找「实体轮廓→终点」的最近点（绕行终点）
            obs_to_target = nearest_points(obs_poly.boundary, Point(target_point))
            end_avoid_point = (obs_to_target[0].x, obs_to_target[0].y)
            
            # 4. 匹配绕行起点/终点在轮廓顶点中的位置（精确匹配+容错）
            start_idx = None
            end_idx = None
            contour_len = len(obs_contour)
            
            # 精确匹配顶点（保留6位小数避免浮点误差）
            for idx, (x, y) in enumerate(obs_contour):
                if (round(x, 6) == round(start_avoid_point[0], 6) and 
                    round(y, 6) == round(start_avoid_point[1], 6)):
                    start_idx = idx
                if (round(x, 6) == round(end_avoid_point[0], 6) and 
                    round(y, 6) == round(end_avoid_point[1], 6)):
                    end_idx = idx
            
            # 精确匹配失败时，取最近的顶点
            if start_idx is None and contour_len > 0:
                start_distances = [
                    math.hypot(p[0]-start_avoid_point[0], p[1]-start_avoid_point[1]) 
                    for p in obs_contour
                ]
                start_idx = start_distances.index(min(start_distances))
            
            if end_idx is None and contour_len > 0:
                end_distances = [
                    math.hypot(p[0]-end_avoid_point[0], p[1]-end_avoid_point[1]) 
                    for p in obs_contour
                ]
                end_idx = end_distances.index(min(end_distances))
            
            # 5. 生成顺时针和逆时针的轮廓路径（修复边界计算问题）
            clockwise_path = []
            counter_path = []
            
            if contour_len > 0 and start_idx is not None and end_idx is not None:
                # 顺时针路径生成
                if start_idx <= end_idx:
                    clockwise_path = obs_contour[start_idx:end_idx+1]
                else:
                    clockwise_path = obs_contour[start_idx:] + obs_contour[:end_idx+1]
                
                # 逆时针路径生成（修复end_idx=0的边界问题）
                if start_idx >= end_idx:
                    counter_path = obs_contour[start_idx:end_idx-1:-1] if end_idx > 0 else obs_contour[start_idx::-1]
                else:
                    counter_part1 = obs_contour[start_idx::-1]  # 从start_idx到0
                    counter_part2 = obs_contour[:end_idx-1:-1] if end_idx > 0 else []  # 从最后到end_idx+1
                    counter_path = counter_part1 + counter_part2
            
            # 6. 计算两个方向的路径长度，选择更短的
            def calc_path_length(path):
                """计算路径总长度"""
                length = 0
                for i in range(len(path)-1):
                    length += calculate_distance(path[i], path[i+1])
                return length
            
            clockwise_len = calc_path_length(clockwise_path)
            counter_len = calc_path_length(counter_path)
            
            # 选择最短的绕行轮廓路径（优先顺时针，长度相同时）
            best_contour_path = clockwise_path if clockwise_len <= counter_len else counter_path
            
            # 7. 拼接绕行路径：当前点 → 绕行起点 → 轮廓路径 → 绕行终点
            if start_avoid_point not in final_route:
                final_route.append(start_avoid_point)
            # 加入轮廓路径（去重）
            for p in best_contour_path:
                p_tuple = (round(p[0], 8), round(p[1], 8))
                last_p = (round(final_route[-1][0], 8), round(final_route[-1][1], 8)) if final_route else None
                if p_tuple != last_p:
                    final_route.append(p)
            if end_avoid_point not in final_route:
                final_route.append(end_avoid_point)
            
            # 更新当前点，继续处理下一个障碍物
            current_point = end_avoid_point
        
        # 8. 加入最终终点
        if target_point not in final_route:
            final_route.append(target_point)
        
        # ========== 路径优化：去重 + 穿透校验 ==========
        # 第一步：去重（保留顺序，8位小数精度）
        final_route_clean = []
        seen = set()
        for point in final_route:
            point_tuple = (round(point[0], 8), round(point[1], 8))
            if point_tuple not in seen:
                seen.add(point_tuple)
                final_route_clean.append(point)
        final_route = final_route_clean
        
        # 第二步：穿透校验 + 紧急外扩（确保所有线段不穿透实体）
        for i in range(len(final_route)-1):
            segment = LineString([final_route[i], final_route[i+1]])
            for obs in solid_obstacles:
                if segment.intersects(obs["poly"]):
                    # 计算线段中点，向远离障碍物中心方向外扩2倍安全距离
                    mid_point = (
                        (final_route[i][0] + final_route[i+1][0])/2,
                        (final_route[i][1] + final_route[i+1][1])/2
                    )
                    # 计算远离障碍物中心的方向向量
                    dx = mid_point[0] - obs["centroid"].x
                    dy = mid_point[1] - obs["centroid"].y
                    dist = math.hypot(dx, dy) or 1  # 避免除零
                    dx_normalized = dx / dist
                    dy_normalized = dy / dist
                    # 外扩点坐标
                    expand_point = (
                        mid_point[0] + dx_normalized * SAFE_DISTANCE * 2,
                        mid_point[1] + dy_normalized * SAFE_DISTANCE * 2
                    )
                    # 插入外扩点，避免穿透
                    final_route.insert(i+1, expand_point)
                    break  # 处理完当前线段，继续下一段
        
        # ========== 计算总路径长度 ==========
        total_distance = 0
        for i in range(len(final_route)-1):
            total_distance += calculate_distance(final_route[i], final_route[i+1])
        total_distance_km = total_distance * 111  # 经纬度转公里（近似值）
        
        # 生成最终状态信息
        status = (
            f"🔴 实体障碍物完整绕行！\n"
            f"无人机高度({drone_h}m) < 障碍物高度，已避开：{','.join(avoid_obstacles)}\n"
            f"安全半径：{st.session_state.get('drone_safety_radius', 50)}米 | "
            f"总路径长度≈{total_distance_km:.3f}公里\n"
            f"路径节点数：{len(final_route)}个（沿实体外围完整绕行）"
        )
        
    except Exception as e:
        # 异常处理
        final_route = [A, B] if A and B else []
        status = f"❌ 路径计算出错：{str(e)}"
    
    return final_route, status