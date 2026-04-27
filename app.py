def calculate_shortest_no_overlap_route():
    """
    修正版：实体多边形障碍物完整外围绕行（非切角）
    核心逻辑：
    1. 实体障碍物视为实心，无人机必须绕完整外轮廓（安全半径外）
    2. 找起点到障碍物轮廓的最近点 → 沿轮廓所有顶点 → 轮廓到终点的最近点
    3. 保证路径最短且不穿透实体
    """
    # 优先使用转换后的坐标
    A = st.session_state.transformed_points["point_a"] or st.session_state.point_a
    B = st.session_state.transformed_points["point_b"] or st.session_state.point_b
    
    if not A or not B:
        return [], "未设置起点A/终点B（地面基准：0米）"
    
    drone_h = st.session_state.drone_height
    SAFE_DISTANCE = get_safe_distance()  # 动态安全距离（经纬度单位）
    final_route = [A]
    avoid_obstacles = []
    
    # 优先使用转换后的障碍物坐标
    obstacles = st.session_state.transformed_points["obstacles"] or st.session_state.obstacles_all
    
    # 过滤需要避开的实体障碍物（高度>无人机高度）
    solid_obstacles = []
    for i, obs_coords in enumerate(obstacles):
        if len(obs_coords) < 3:
            continue
        
        obs_h = st.session_state.obstacles_height[i] if i < len(st.session_state.obstacles_height) else 50
        obs_type = st.session_state.obstacles_type[i] if i < len(st.session_state.obstacles_type) else "自定义障碍物"
        
        if obs_h <= drone_h:
            continue
        
        avoid_obstacles.append(obs_type)
        # 1. 构建实体障碍物多边形
        obs_poly = Polygon(obs_coords)
        # 2. 按安全半径外扩（保证无人机在实体外安全距离）
        scale_factor = 1.0 + (st.session_state.drone_safety_radius / 1000)  # 按安全半径比例外扩
        obs_poly_safe = scale(obs_poly, xfact=scale_factor, yfact=scale_factor, origin='centroid')
        # 3. 提取实体障碍物的完整外轮廓顶点（闭合前的所有顶点）
        contour_vertices = list(obs_poly_safe.exterior.coords)
        if contour_vertices[0] == contour_vertices[-1]:
            contour_vertices = contour_vertices[:-1]  # 移除重复的闭合点
        
        solid_obstacles.append({
            "poly": obs_poly_safe,       # 安全距离外的实体多边形
            "contour": contour_vertices, # 实体外轮廓顶点列表
            "type": obs_type,
            "height": obs_h,
            "centroid": obs_poly_safe.centroid  # 实体中心（用于判断绕行方向）
        })
    
    if not solid_obstacles:
        # 无需要避开的实体障碍物，直接直线飞行
        final_route = [A, B]
        status = f"🟢 最短直线飞行！无人机高度({drone_h}m) ≥ 所有实体障碍物高度，直接从A到B（安全半径：{st.session_state.drone_safety_radius}米）"
        return final_route, status
    
    # ========== 核心：实体障碍物完整绕行逻辑 ==========
    current_point = A
    target_point = B
    
    # 遍历所有需要绕行的实体障碍物（这里先处理单个，多障碍物可扩展）
    for obs in solid_obstacles:
        obs_poly = obs["poly"]
        obs_contour = obs["contour"]
        obs_centroid = obs["centroid"]
        
        # 1. 检查当前点→终点的直线是否穿透实体障碍物
        direct_line = LineString([current_point, target_point])
        if not direct_line.intersects(obs_poly):
            continue  # 不穿透，无需绕行
        
        # 2. 找「当前点→实体轮廓」的最近点（绕行起点）
        current_to_obs = nearest_points(Point(current_point), obs_poly.boundary)
        start_avoid_point = (current_to_obs[1].x, current_to_obs[1].y)
        
        # 3. 找「实体轮廓→终点」的最近点（绕行终点）
        obs_to_target = nearest_points(obs_poly.boundary, Point(target_point))
        end_avoid_point = (obs_to_target[0].x, obs_to_target[0].y)
        
        # 4. 确定绕行方向（沿轮廓顺时针/逆时针，保证路径最短）
        # 计算两个方向的路径长度：顺时针绕轮廓 vs 逆时针绕轮廓
        start_idx = None
        end_idx = None
        # 匹配绕行起点/终点在轮廓顶点中的位置
        for idx, (x, y) in enumerate(obs_contour):
            if round(x, 6) == round(start_avoid_point[0], 6) and round(y, 6) == round(start_avoid_point[1], 6):
                start_idx = idx
            if round(x, 6) == round(end_avoid_point[0], 6) and round(y, 6) == round(end_avoid_point[1], 6):
                end_idx = idx
        
        # 如果没精确匹配到顶点，取最近的顶点
        if start_idx is None:
            start_distances = [math.hypot(p[0]-start_avoid_point[0], p[1]-start_avoid_point[1]) for p in obs_contour]
            start_idx = start_distances.index(min(start_distances))
        if end_idx is None:
            end_distances = [math.hypot(p[0]-end_avoid_point[0], p[1]-end_avoid_point[1]) for p in obs_contour]
            end_idx = end_distances.index(min(end_distances))
        
        # 生成顺时针和逆时针的轮廓路径
        contour_len = len(obs_contour)
        # 顺时针路径
        if start_idx <= end_idx:
            clockwise_path = obs_contour[start_idx:end_idx+1]
        else:
            clockwise_path = obs_contour[start_idx:] + obs_contour[:end_idx+1]
        # 逆时针路径
        if start_idx >= end_idx:
            counter_path = obs_contour[start_idx:end_idx-1:-1] if end_idx > 0 else obs_contour[start_idx::-1]
        else:
            counter_path = obs_contour[start_idx::-1] + obs_contour[:end_idx-1:-1] if end_idx > 0 else obs_contour[start_idx::-1]
        
        # 计算两个方向的路径长度，选更短的
        def calc_path_length(path):
            length = 0
            for i in range(len(path)-1):
                length += math.hypot(path[i][0]-path[i+1][0], path[i][1]-path[i+1][1])
            return length
        
        clockwise_len = calc_path_length(clockwise_path)
        counter_len = calc_path_length(counter_path)
        
        # 选择最短的绕行轮廓路径
        if clockwise_len <= counter_len:
            best_contour_path = clockwise_path
        else:
            best_contour_path = counter_path
        
        # 5. 拼接完整绕行路径：当前点 → 绕行起点 → 轮廓路径 → 绕行终点
        final_route.append(start_avoid_point)
        final_route.extend(best_contour_path)  # 加入完整轮廓顶点（沿实体外围走）
        final_route.append(end_avoid_point)
        
        # 更新当前点，继续处理下一个障碍物
        current_point = end_avoid_point
    
    # 6. 加入终点，完成路径
    final_route.append(target_point)
    
    # ========== 路径优化：去重 + 校验（确保不穿透实体） ==========
    # 去重（保留顺序）
    final_route_clean = []
    seen = set()
    for point in final_route:
        point_tuple = (round(point[0], 8), round(point[1], 8))
        if point_tuple not in seen:
            seen.add(point_tuple)
            final_route_clean.append(point)
    final_route = final_route_clean
    
    # 最终校验：确保所有线段都不穿透实体障碍物
    for i in range(len(final_route)-1):
        segment = LineString([final_route[i], final_route[i+1]])
        for obs in solid_obstacles:
            if segment.intersects(obs["poly"]):
                # 紧急外扩：在穿透点外增加安全距离
                mid_point = (
                    (final_route[i][0] + final_route[i+1][0])/2,
                    (final_route[i][1] + final_route[i+1][1])/2
                )
                # 远离实体中心方向外扩
                dx = mid_point[0] - obs["centroid"].x
                dy = mid_point[1] - obs["centroid"].y
                dist = math.hypot(dx, dy) or 1
                dx /= dist
                dy /= dist
                expand_point = (
                    mid_point[0] + dx * SAFE_DISTANCE * 2,
                    mid_point[1] + dy * SAFE_DISTANCE * 2
                )
                final_route.insert(i+1, expand_point)
                break
    
    # 计算总路径长度
    total_distance = 0
    for i in range(len(final_route)-1):
        total_distance += calculate_distance(final_route[i], final_route[i+1])
    total_distance_km = total_distance * 111  # 经纬度转公里（近似）
    
    status = f"🔴 实体障碍物完整绕行！无人机高度({drone_h}m) < 障碍物高度，已避开：{','.join(avoid_obstacles)}，总路径长度≈{total_distance_km:.3f}公里（安全半径：{st.session_state.drone_safety_radius}米，沿实体外围完整绕行）"
    return final_route, status