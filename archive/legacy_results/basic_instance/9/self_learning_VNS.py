import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from copy import deepcopy
import random
import os
import math
# 关闭warnings
import warnings
warnings.filterwarnings("ignore")
os.environ['OMP_NUM_THREADS'] = '1'
os.environ['MKL_NUM_THREADS'] = '1'

def get_drone_route_time(drone_route, tij_d, start_time, inspection_times):
    drone_arrive_time = []
    drone_leave_time = []
    # print(drone_route)
    for index, node in enumerate(drone_route):
        if node == drone_route[0]:
            drone_arrive_time.append(start_time)
            drone_leave_time.append(start_time + inspection_times[node])
        elif node != drone_route[-1]:
            travel_time = tij_d[drone_route[index - 1]][node]
            drone_arrive_time.append(drone_leave_time[index - 1] + travel_time)
            drone_leave_time.append(drone_arrive_time[index] + inspection_times[node])
        else:
            travel_time = tij_d[drone_route[index - 1]][node]
            drone_arrive_time.append(drone_leave_time[index - 1] + travel_time)
            drone_leave_time.append(drone_arrive_time[index] + inspection_times[node])
            
    return {
        'arrival_time': drone_arrive_time,
        'leave_time': drone_leave_time,
        'route_time': drone_arrive_time[-1] - drone_leave_time[0] + inspection_times[drone_route[-1]]
    }

def get_total_time(drone_route, truck_route, tij_k, tij_d, V_input, inspection_times, LTD, RTD):
    """
    drone_route = {
        1:{1:[0,2,9,10,3], 2:[1,4,5]},
        2:{1:[0,7,3]}
    }
    第一层表示无人机, 第二次表示该无人机的sortie
    
    drone_time_schedule = {
        arrival_time: [0, 2, 9, 10, 30],
        leave_time: [0, 2, 9, 10, 30],
        route_time: 30
    }
    
    drone_arrival_time = {
        1:{1:[1,2,3], 2:[4,5]}, 
        2:{1:[7,9]}
        }
    
    drone_launch_node = {
        1:[0,2,3,5,7],
        2:[0,7]
    }
    
    从直觉上来说
    无人机降落时间不包含RTD回收时间
    无人机起飞时间包含LTD起飞时间
    """
    truck_leave_time = {}
    truck_arrival_time = {}
    drone_leave_time = {}
    drone_arrival_time = {}
    drone_route_time = {}
    V = deepcopy(V_input)
    for d in V_input:
        drone_arrival_time[d] = {}
        drone_leave_time[d] = {}
        drone_route_time[d] = {}
        if drone_route.get(d) is None:
            V.remove(d)
            continue
    drone_launch_node = {}
    drone_arrival_node = {}
    total_launch_node = set()
    total_arrival_node = set()
    # print(drone_route[d])
    for d in V:
        launch = []
        arrival = []
        # print(drone_route)
        
        for sortie in drone_route[d].values():
            launch.append(sortie[0])
            arrival.append(sortie[-1])
        drone_launch_node[d] = launch
        drone_arrival_node[d] = arrival
        total_launch_node.update(launch)
        total_arrival_node.update(arrival)
        
    for idx, node in enumerate(truck_route):
        # # 如果第一个卡车点是起点，则设置起点的离开时间和到达时间为0
        # if node == truck_route[0]:
        #     truck_leave_time[node] = 0
        #     truck_arrival_time[node] = 0
            
        # 如果没有无人机起降
        if (node not in total_launch_node) and (node not in total_arrival_node):
            if node == truck_route[0]:
                truck_leave_time[node] = 0
                truck_arrival_time[node] = 0
                continue
            truck_arrival_time[node] = truck_leave_time[truck_route[idx - 1]] + tij_k[truck_route[idx - 1]][node]
            truck_leave_time[node] = truck_arrival_time[node]

        # 如果该点是无人机起飞点，且先前没有更新卡车的离开时间和到达时间，并且找到无人机到达点对应的sortie，使用函数get_drone_route_time计算无人机的到达时间和离开时间并更新
        if (node in total_launch_node) and (node not in total_arrival_node):
            if node == truck_route[0]:
                truck_leave_time[node] = LTD
                truck_arrival_time[node] = 0
            else:
                truck_arrival_time[node] = truck_leave_time[truck_route[idx - 1]] + tij_k[truck_route[idx - 1]][node]
                truck_leave_time[node] = truck_arrival_time[node] + LTD
            for d in V:
                for idx_sortie in list(drone_route[d].items()):
                    sortie = idx_sortie[1]
                    index = idx_sortie[0]
                    if sortie[0] == node:
                        drone_time_schedule = get_drone_route_time(drone_route[d][index], tij_d, truck_leave_time[sortie[0]], inspection_times)
                        drone_arrival_time[d][index] = drone_time_schedule['arrival_time']
                        drone_leave_time[d][index] = drone_time_schedule['leave_time']
                        drone_route_time[d][index] = drone_time_schedule['route_time']

        # 如果该点是无人机到达点，更新卡车的离开时间和到达时间
        if (node in total_arrival_node) and (node not in total_launch_node):
            # 用于记录所有从这个点起飞的无人机编号与sortie索引
            drone_sorties = []
            for d in V:
                for idx_sortie in list(drone_route[d].items()):
                    sortie = idx_sortie[1]
                    index = idx_sortie[0]
                    if sortie[-1] == node:
                        drone_sorties.append((d, index))
        
            all_drone_arrival_time = [drone_arrival_time[d][inner_idx][-1] for d, inner_idx in drone_sorties]
            truck_arrival_time[node] = tij_k[truck_route[idx - 1]][node] + truck_leave_time[truck_route[idx - 1]]
            truck_leave_time[node] = max(max(all_drone_arrival_time), truck_arrival_time[node]) + RTD
            
        # 如果该点既是无人机到达点又是无人机起飞点
        if (node in total_launch_node) and (node in total_arrival_node):
            truck_arrival_time[node] = truck_leave_time[truck_route[idx - 1]] + tij_k[truck_route[idx - 1]][node]
            all_drone_arrival_time = []
            all_drone_leave_time = []
            for d in V:
                # 如果这个点既是无人机d前一个sortie的到达点，又是无人机d后一个sortie的起飞点
                if (node in drone_launch_node[d]) and (node in drone_arrival_node[d]):
                    drone_former_sorties = []
                    for idx_sortie in list(drone_route[d].items()):
                        sortie = idx_sortie[1]
                        index = idx_sortie[0]
                        # 提取上一个sortie无人机在该访问点的到达时间和离开时间
                        if sortie[-1] == node:
                            drone_former_sorties.append((d, index))

                    for idx_sortie in list(drone_route[d].items()):
                        sortie = idx_sortie[1]
                        index = idx_sortie[0]
                        if sortie[0] == node:
                            drone_time_schedule = get_drone_route_time(drone_route[d][index], tij_d, max(drone_arrival_time[drone_former_sorties[0][0]][drone_former_sorties[0][1]][-1], truck_arrival_time[node])+RTD+LTD, inspection_times)
                            drone_arrival_time[d][index] = drone_time_schedule['arrival_time']
                            drone_leave_time[d][index] = drone_time_schedule['leave_time']
                            drone_route_time[d][index] = drone_time_schedule['route_time']
                            all_drone_leave_time.append(drone_time_schedule['leave_time'][0])
                # 如果这个点是无人机d的到达点, 且无人机不起飞
                if (node in drone_arrival_node[d]) and (node not in drone_launch_node[d]):
                    for idx_sortie in list(drone_route[d].items()):
                        sortie = idx_sortie[1]
                        index = idx_sortie[0]
                        if sortie[-1] == node:
                            all_drone_arrival_time.append(max(drone_arrival_time[d][index][-1], truck_arrival_time[node])+RTD)
                # 无人机在这个点只起飞，先前并没有在这个点降落
                if (node in drone_launch_node[d]) and (node not in drone_arrival_node[d]):
                    for idx_sortie in list(drone_route[d].items()):
                        sortie = idx_sortie[1]
                        index = idx_sortie[0]
                        if sortie[0] == node:
                            drone_time_schedule = get_drone_route_time(drone_route[d][index], tij_d, truck_arrival_time[node]+LTD, inspection_times)
                            drone_arrival_time[d][index] = drone_time_schedule['arrival_time']
                            drone_leave_time[d][index] = drone_time_schedule['leave_time']
                            drone_route_time[d][index] = drone_time_schedule['route_time']
                            all_drone_leave_time.append(drone_time_schedule['leave_time'][0])
                total_operation_time = all_drone_arrival_time + all_drone_leave_time + [truck_arrival_time[node]]
                truck_leave_time[node] = max(total_operation_time)
    return truck_leave_time, truck_arrival_time, drone_arrival_time, drone_leave_time, drone_route_time


def cluster_coverage_points(N_coord, total_area_line_coverage_points, total_point_coverage_points, num):
    pairs = {}
    pair_index_list = []
    point_index_list = []
    pair_index = 0
    for i in range(0, len(total_area_line_coverage_points), 2):
        pairs[pair_index] = [total_area_line_coverage_points[i], total_area_line_coverage_points[i+1]]
        pair_index_list.append(pair_index)
        pair_index += 1
    N_coord_pair_center = np.zeros([len(pair_index_list)+len(total_point_coverage_points), 2])
    for pair_index in pair_index_list:
        N_coord_pair_center[pair_index,0] = (N_coord[pairs[pair_index][0]][0] + N_coord[pairs[pair_index][1]][0])/2
        N_coord_pair_center[pair_index,1] = (N_coord[pairs[pair_index][0]][1] + N_coord[pairs[pair_index][1]][1])/2
    for point_index in range(len(pair_index_list), len(pair_index_list)+len(total_point_coverage_points)):
        N_coord_pair_center[point_index, 0] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][0]
        N_coord_pair_center[point_index, 1] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][1]
        point_index_list.append(point_index)
    # 使用KMeans聚类算法将覆盖点分成|num|个簇
    kmeans = KMeans(n_clusters=num)
    kmeans.fit(N_coord_pair_center)
    labels = kmeans.labels_
    clustered_points = {}
    for i in range(num):
        clustered_points[i+1] = [[]]
    for idx, label in enumerate(labels):
        if idx < len(pair_index_list):
            clustered_points[label+1][0].append(pairs[idx][0])
            clustered_points[label+1][0].append(pairs[idx][1])
        else:
            clustered_points[label+1][0].append(total_point_coverage_points[idx-len(pair_index_list)])
    return clustered_points

def check_drone_route_feasible(drone_route,DD, tij_d, inspection_times):
    if len(drone_route) == 1:
        return True
    route_time = get_drone_route_time(drone_route, tij_d, 0, inspection_times)['route_time']
    if route_time <= DD:
        return True
    return False

def get_drone_truck_route_by_clustering(N_coord, N1, N2, N3, total_area_line_coverage_points, total_point_coverage_points, DD, V, tij_d, inspection_times, dij):
    """
    clustered_points = {1: [[5, 6, 19], [10, 16, 17, 18], [7, 8, 9, 14, 15], [11, 12, 13]]}
    """
    # 首先使用聚类算法得到|V|个coverage points集合
    clustered_points = cluster_coverage_points(N_coord, total_area_line_coverage_points, total_point_coverage_points, len(V))
    feasible = False
    # print(clustered_points)
    while not feasible:
        infeasible_times = 0
        for drone_id in clustered_points.keys():
            # print(clustered_points)
            for route in clustered_points[drone_id]:
                # print(route)
                if not check_drone_route_feasible(route, DD, tij_d, inspection_times):
                    # 获得route中的total_area_line_coverage_points的index
                    # print(route)
                    line_coverage_points = [point for point in route if point in total_area_line_coverage_points]
                    point_coverage_points = [point for point in route if point in total_point_coverage_points]
                    # print(line_coverage_points)
                    # print(point_coverage_points)
                    # 如果不满足约束条件，进行调整
                    
                    split_routes = cluster_coverage_points(N_coord, line_coverage_points, point_coverage_points, 2)
                    # print(split_routes)
                    split_route_1 = split_routes[1][0]
                    split_route_2 = split_routes[2][0]
                    # print(split_route_1)
                    # print(split_route_2)
                    clustered_points[drone_id].remove(route)
                    clustered_points[drone_id].append(split_route_1)
                    clustered_points[drone_id].append(split_route_2)
                    # print(clustered_points)
                    infeasible_times += 1
        if infeasible_times == 0:
            feasible = True
    # print(clustered_points)
    # 假设卡车的无人机起降点个数大于等于无人机出动次数，现在需要为无人机每一次出动安排起降点，并且同一架无人机的起降点不能相同，同一架无人机不同的sortie的起飞点、降落点也不能相同
    # 首先计算路径的起飞点,并给所有无人机路径设置最近的起飞点
    # print(clustered_points)
    # still_need_operational_node = True
    
    copy_clustered_points = deepcopy(clustered_points)
    # print("Initial clustered points:", clustered_points)
    has_fatal_problem = True
    while has_fatal_problem:
        # N3生成访问顺序先后的字典pij=1表示i在j之前被访问,默认是-1，key是(i,j)
        pij = {(i,j): -1 for i in N3 for j in N3}
        has_fatal_problem = False
        for drone_id, routes in clustered_points.items():
            if has_fatal_problem == True:
                break
            used_launch_nodes = set()
            used_retrieved_nodes = set()
            while True:
                if has_fatal_problem == True:
                    break
                need_num = 0
                for route in routes:
                    if route[0] not in N1 and route[-1] not in N2:
                        need_num += 1
                        # print(route)
                if need_num == 0:
                    break
                
                # for route in routes:
                idx=0
                while idx < len(routes):
                    # print(idx)
                    route = routes[idx]
                    if has_fatal_problem == True:
                        break
                    # print(route)
                    if route[0] in N1 and route[-1] in N2:
                        continue
                    shortest_node = [0, 0]
                    shortest_distance = np.inf
                    
                    all_infeasible = True
                    while all_infeasible:
                        for launch_node in N1:
                            if launch_node in used_launch_nodes:
                                continue
                            for retrieved_node in N2:
                                # if pij[(launch_node, retrieved_node)] == 0:
                                #     continue
                                if retrieved_node in used_retrieved_nodes:
                                    continue
                                if launch_node == retrieved_node:
                                    continue
                            # 如果时间超过无人机续航，则continue
                                if get_drone_route_time([launch_node]+route+[retrieved_node], tij_d, 0, inspection_times)['route_time'] > DD:
                                    # print([launch_node]+route+[retrieved_node])
                                    continue
                                distance_1 = dij[launch_node, route[0]] + dij[retrieved_node, route[-1]]
                                distance_2 = dij[launch_node, route[-1]] + dij[retrieved_node, route[0]]
                                if distance_1 < distance_2:
                                    reverse_or_not = False
                                    distance = distance_1
                                else:
                                    reverse_or_not = True
                                    distance = distance_2
                                if distance < shortest_distance:
                                    shortest_distance = distance
                                    shortest_node[0] = launch_node
                                    shortest_node[1] = retrieved_node
                                    # pij[(launch_node, retrieved_node)] = 1
                                all_infeasible = False
                        # print(all_infeasible)    
                        # 如果卡车出发点全都会使得其不可行，那么再次分割该route
                        if all_infeasible:
                            if len(route) == 1 and route[0] not in total_point_coverage_points:
                                raise Exception("A single coverage point cannot be served within the drone's endurance limit.")
                            
                            line_coverage_points = [point for point in route if point in total_area_line_coverage_points]
                            point_coverage_points = [point for point in route if point in total_point_coverage_points]
                            if len(line_coverage_points) == 2 and len(point_coverage_points) == 0:
                                # 如果只有一对覆盖点那么重新运行这个循环并把clustered_points通过copy_clustered_points还原
                                clustered_points = deepcopy(copy_clustered_points)
                                has_fatal_problem = True
                                break
                            split_routes = cluster_coverage_points(N_coord, line_coverage_points, point_coverage_points, 2)
                            # print(split_routes)
                            # return
                            split_route_1 = split_routes[1][0]
                            split_route_2 = split_routes[2][0]
                            routes.remove(route)
                            routes.append(split_route_1)
                            routes.append(split_route_2)
                            break
                        else:
                            pij[(shortest_node[0], shortest_node[1])] = 1
                            pij[(shortest_node[1], shortest_node[0])] = 0
                            if reverse_or_not:
                                route.reverse()
                                route.insert(0, shortest_node[0])
                                route.append(shortest_node[1])
                                used_launch_nodes.add(shortest_node[0])
                                used_retrieved_nodes.add(shortest_node[1])
                                idx += 1
                            else:
                                route.insert(0, shortest_node[0])
                                route.append(shortest_node[1])
                                used_launch_nodes.add(shortest_node[0])
                                used_retrieved_nodes.add(shortest_node[1])
                                idx += 1
            # 检查clustered_points中每架无人机路径的起飞降落点是否闭环（即起飞点集合等于降落点集合）
            all_retrieved_node = set()
            all_launch_node = set()
            for route in routes:
                all_launch_node.add(route[0])
                all_retrieved_node.add(route[-1])
            if all_launch_node == all_retrieved_node:
                has_fatal_problem = True
    # print("Final clustered points:", clustered_points)
    # 如果clustered_points
    # 下面对卡车路径的访问顺序进行研究：
    # print(clustered_points)
    truck_route_by_drone = {}
    for d in V:
        truck_route_by_drone[d] = []
    for d in V:
        all_launch_node = set()
        all_retrieved_node = set()
        launch_retrieve_pair = {}
        retrieve_launch_pair = {}
        for route in clustered_points[d]:
            all_launch_node.add(route[0])
            all_retrieved_node.add(route[-1])
            launch_retrieve_pair[route[0]] = route[-1]
            retrieve_launch_pair[route[-1]] = route[0]
        # last_second_node = None
        last_node = None
        # print(clustered_points[d])
        # print(all_launch_node)
        for node in all_launch_node:
            # print(node)
            # print(truck_route_by_drone[d])
            if node == N3[0]:
                truck_route_by_drone[d].insert(0, node)
                if launch_retrieve_pair[node] == N3[-1]:
                    last_node = N3[-1]
                    continue
                truck_route_by_drone[d].insert(1, launch_retrieve_pair[node])
                continue
            if node not in all_retrieved_node:
                if launch_retrieve_pair[node] in truck_route_by_drone[d]:
                    idx = truck_route_by_drone[d].index(launch_retrieve_pair[node])
                    truck_route_by_drone[d].insert(idx, node)
                    continue
                    
                truck_route_by_drone[d].append(node)
                if launch_retrieve_pair[node] == N3[-1]:
                    last_node = N3[-1]
                    continue
                truck_route_by_drone[d].append(launch_retrieve_pair[node])
                continue
            if node in all_retrieved_node and node in truck_route_by_drone[d]:
                if launch_retrieve_pair[node] == N3[-1]:
                    last_node = N3[-1]
                    continue
                if launch_retrieve_pair[node] in truck_route_by_drone[d]:
                    continue
                truck_route_by_drone[d].append(launch_retrieve_pair[node])
                continue
            if node in all_retrieved_node and node not in truck_route_by_drone[d]:
                if launch_retrieve_pair[node] in truck_route_by_drone[d]:
                    idx = truck_route_by_drone[d].index(launch_retrieve_pair[node])
                    truck_route_by_drone[d].insert(idx, node)
                    # print(f'index:{idx}')
                    continue
                truck_route_by_drone[d].append(node)
                if launch_retrieve_pair[node] == N3[-1]:
                    last_node = N3[-1]
                    continue
                truck_route_by_drone[d].append(launch_retrieve_pair[node])
                continue
        if last_node is not None:
            truck_route_by_drone[d].append(last_node)
    # 把一个卡车路径当作基准，用其他无人机的卡车路径进行插入对其，以生成一辆卡车的路径
    base_truck_route = deepcopy(truck_route_by_drone[1])
    # print("Truck route by drone before merging:", truck_route_by_drone) 
    if base_truck_route[0] != N3[0]:
        base_truck_route.insert(0, N3[0])
    if base_truck_route[-1] != N3[-1]:
        base_truck_route.insert(len(base_truck_route), N3[-1])
    # 获得可供插入base_truck_route的index
    for d in V:
        
        if d == 1:
            continue
        else:

            # 记录上个插入点的位置，后面的点需要在这个插入点之后找位置插入
            last_insert_position = 1
            for index, node in enumerate(truck_route_by_drone[d]):
                
                if node not in base_truck_route:
                    after_node_index = len(base_truck_route)-1
                    # 寻找后面的node，如果最近的一个后方node在base_truck_route里，那就记录其位置，该node的插入位置需要在已记录位置之前
                    for i in range(index+1, len(truck_route_by_drone[d])):
                        after_node = truck_route_by_drone[d][i]
                        if after_node in base_truck_route:
                            after_node_index = base_truck_route.index(after_node)
                            # print(after_node_index)
                            break
                    # 找到距离node最近的base_truck_route中的点
                    nearest_base_node = None
                    nearest_distance = np.inf
                    for base_node in base_truck_route[last_insert_position:after_node_index+1]:
                        distance = np.linalg.norm(N_coord[node]-N_coord[base_node])
                        if distance < nearest_distance:
                            nearest_distance = distance
                            nearest_base_node = base_node
                    # if node==5:
                    # print(last_insert_position)
                    # print(after_node_index+1)
                    # print(base_truck_route)
                    # print(node)
                    # 找到nearest_base_node在base_truck_route中的index
                    # print(last_insert_position)
                    # print(after_node_index+1)
                    # if nearest_base_node is None:
                    #     print(node)
                    #     print(last_insert_position)
                    #     print(after_node_index+1)
                    #     print(truck_route_by_drone)
                    nearest_base_node_index = base_truck_route.index(nearest_base_node)
                    # 在nearest_base_node_index前后插入node，并且选择使得卡车路径增加距离最小的方式进行插入
                    if nearest_base_node_index == 0:
                        base_truck_route.insert(0, node)
                        last_insert_position = 0
                    elif nearest_base_node_index == len(base_truck_route)-1:
                        base_truck_route.insert(nearest_base_node_index, node)
                        last_insert_position = len(base_truck_route)-1
                    elif nearest_base_node_index == after_node_index:
                        base_truck_route.insert(nearest_base_node_index, node)
                        last_insert_position = nearest_base_node_index
                    elif nearest_base_node_index == last_insert_position:
                        base_truck_route.insert(nearest_base_node_index+1, node)
                        last_insert_position = nearest_base_node_index + 1
                    else:
                        before_node = base_truck_route[nearest_base_node_index-1]
                        after_node = base_truck_route[nearest_base_node_index+1]
                        increase_distance_if_before = np.linalg.norm(N_coord[before_node]-N_coord[node]) + np.linalg.norm(N_coord[node]-N_coord[nearest_base_node]) - np.linalg.norm(N_coord[before_node]-N_coord[nearest_base_node])
                        increase_distance_if_after = np.linalg.norm(N_coord[after_node]-N_coord[node]) + np.linalg.norm(N_coord[node]-N_coord[nearest_base_node]) - np.linalg.norm(N_coord[after_node]-N_coord[nearest_base_node])
                        if increase_distance_if_before < increase_distance_if_after:
                            base_truck_route.insert(nearest_base_node_index, node)
                            last_insert_position = nearest_base_node_index
                        else:
                            base_truck_route.insert(nearest_base_node_index+1, node)
                            last_insert_position = nearest_base_node_index + 1  
                if node in base_truck_route:
                    last_insert_position = base_truck_route.index(node) + 1
                    # print(base_truck_route)
    results_drone_route = {}
    for d in V:
        results_drone_route[d] = {}
        for idx, route in enumerate(clustered_points[d]):
            results_drone_route[d][idx+1] = route
    # 在保证访问顺序的前提下对truck_route进行贪心优化
    current_node = base_truck_route[0]
    # print("Base truck route before optimization:", base_truck_route)
    # print(pij)
    new_truck_route = [current_node]
    all_node_num = len(base_truck_route)
    while len(new_truck_route) < all_node_num-1:
        lowest_distance = np.inf
        best_next_node = None
        for next_node in base_truck_route[1:-1]:
            # print(next_node)
            # print(new_truck_route)
            to_continue = False
            if next_node in new_truck_route:
                to_continue = True
            if pij[(current_node, next_node)] == 0:
                to_continue = True
            for next_node_sortie_former in base_truck_route:
                if pij[(next_node_sortie_former, next_node)] == 1 and next_node_sortie_former not in new_truck_route:
                    to_continue = True
            if to_continue:
                continue
            distance = dij[current_node][next_node]
            if distance < lowest_distance:
                lowest_distance = distance
                best_next_node = next_node
        new_truck_route.append(best_next_node)
        current_node = best_next_node
    new_truck_route.append(base_truck_route[-1])
        
    return results_drone_route, new_truck_route, truck_route_by_drone



def initialization(N_coord, N1, N2, N3, N_s, N_p, V, dij, tij_k, tij_d, S, DD, is_point, inspection_times, LTD, RTD):
    try_again = True
    while try_again:
        try_again = False
        total_coverage_points = []
        for s in S:
            total_coverage_points = total_coverage_points + N_s[s]
        total_area_line_coverage_points = []
        total_point_coverage_points = []
        for s in S:
            if is_point[s] == 0:
                total_area_line_coverage_points = total_area_line_coverage_points + N_s[s]
            else:
                total_point_coverage_points = total_point_coverage_points + N_s[s]
        drone_route, truck_route, truck_route_by_drone = get_drone_truck_route_by_clustering(N_coord, N1, N2, N3, total_area_line_coverage_points, total_point_coverage_points, DD, V, tij_d, inspection_times, dij)
        
        print(drone_route, truck_route)
        try:
            total_time = get_total_time(drone_route, truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)
        except KeyError as e:
            try_again = True
            print(f"KeyError encountered: {e}. Retrying initialization...")
    return truck_route, drone_route, total_time


def check_drone_route_availibility(drone_route, tij_d, inspection_times, DD):
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            drone_route_time = get_drone_route_time(route, tij_d, 0, inspection_times)['route_time']
            if drone_route_time > DD:
                return False
    return True


# NS1 random change two nodes in the truck route.
# Attention should be paid to the impact on the drone route where the launch node and
# the retrieved node is located on the changed nodes. They need to be swapped.

"""
drone_route = {
        1:{1:[0,2,9,10,3], 2:[1,4,5]},
        2:{1:[0,7,3]}
    }
NS_1 需要确保无人机sortie在续航范围内
"""
def truck_route_node_swap(input_truck_route, input_drone_route, node1, node2):
    truck_route = deepcopy(input_truck_route)
    drone_route = deepcopy(input_drone_route)
    # Swap the nodes in the truck route
    idx1 = truck_route.index(node1)
    idx2 = truck_route.index(node2)
    truck_route[idx1], truck_route[idx2] = truck_route[idx2], truck_route[idx1]

    # Update the drone route accordingly
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            for i in range(len(route)):
                if route[i] == node1:
                    route[i] = node2
                elif route[i] == node2:
                    route[i] = node1

    return truck_route, drone_route
def NS1(input_truck_route, input_drone_route, tij_k, tij_d, V, inspection_times, LTD, RTD, DD):
    truck_route = deepcopy(input_truck_route)
    drone_route = deepcopy(input_drone_route)
    shortest_time = get_total_time(drone_route, truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)[0][truck_route[-1]]
    record_truck_route = None
    record_drone_route = None
    for idx_1 in range(1, len(truck_route)-2):
        for idx_2 in range(idx_1+1, len(truck_route)-1):
            if idx_1 == 0 or idx_2 == len(truck_route)-1:
                continue
            node1 = truck_route[idx_1]
            node2 = truck_route[idx_2]
            final_truck_route, final_drone_route = truck_route_node_swap(truck_route, drone_route, node1, node2)
            total_time = get_total_time(final_drone_route, final_truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)[0][final_truck_route[-1]]
            if total_time < shortest_time and total_time <= DD:
                shortest_time = total_time
                record_truck_route = final_truck_route
                record_drone_route = final_drone_route
    if record_truck_route is None:
        record_truck_route = truck_route
    if record_drone_route is None:
        record_drone_route = drone_route
    return record_truck_route, record_drone_route

# NS2 reverse the visit order of the drone route (for every sortie, choose the one that can save the most time of a single trip)
def NS2(input_truck_route, input_drone_route, tij_d, tij_k, inspection_times, DD, V):
    # 使用get_total_time()函数计算总时间
    truck_route = deepcopy(input_truck_route)
    drone_route = deepcopy(input_drone_route)
    # We have three ways for the drone route reversion. One is reverse the total drone route(except for the launch and retrieve node), the second
    # is reverse the first two nodes and the third is reverse the last two nodes
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            drone_route_time = get_drone_route_time(route, tij_d, 0, inspection_times)['route_time']
            if len(route) <= 3:
                continue
            # Reverse the total drone route (except for the launch and retrieve node)
            new_route = [route[0]] + route[1:-1][::-1] + [route[-1]]
            if get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time'] <= DD:
                new_drone_route = deepcopy(drone_route)
                new_drone_route[drone_id][sortie_id] = new_route
                new_route_time = get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time']
                if new_route_time < drone_route_time:
                    drone_route = new_drone_route
                    drone_route_time = new_route_time
            # Reverse the first two nodes (except for the launch and retrieve node)
            if len(route) > 3:
                new_route = [route[0]] + [route[2], route[1]] + route[3:] 
                if get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time'] <= DD:
                    new_drone_route = deepcopy(drone_route)
                    new_drone_route[drone_id][sortie_id] = new_route
                    new_route_time = get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time']
                    if new_route_time < drone_route_time:
                        drone_route = new_drone_route
                        drone_route_time = new_route_time
            # Reverse the last two nodes (except for the launch and retrieve node)
            if len(route) > 4:
                new_route = route[:-3] + [route[-2], route[-3]] + [route[-1]]
                if get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time'] <= DD:
                    new_drone_route = deepcopy(drone_route)
                    new_drone_route[drone_id][sortie_id] = new_route
                    new_route_time = get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time']
                    if new_route_time < drone_route_time:
                        drone_route = new_drone_route
                        drone_route_time = new_route_time
    # total_time = get_total_time(drone_route, truck_route, tij_k, tij_d, V, inspection_times)[0][truck_route[-1]]
    return truck_route, drone_route

# NS3 对于每一个无人机覆盖任务，按照贪婪算法对覆盖点的访问顺序进行重新排序（从起降点开始依次计算点之间的距离，依次访问距离最近的点），选择能够使得总的任务完成时间降低最多的方法
# 排序的时候不按照点排序，按照pair的形式进行排序，然后再扩展成逐个点
def greedy_reorder(N_coord, route, total_area_line_coverage_points, total_point_coverage_points):      
    pairs = {}
    point_match = {}
    pair_index_list = []
    point_index_list = []
    pair_index = 0
    for i in range(0, len(total_area_line_coverage_points), 2):
        pairs[pair_index] = [total_area_line_coverage_points[i], total_area_line_coverage_points[i+1]]
        pair_index_list.append(pair_index)
        pair_index += 1
    # print(pairs)
    N_coord_pair_center = np.zeros([len(pair_index_list)+len(total_point_coverage_points), 2])
    for pair_index in pair_index_list:
        N_coord_pair_center[pair_index,0] = (N_coord[pairs[pair_index][0]][0] + N_coord[pairs[pair_index][1]][0])/2
        N_coord_pair_center[pair_index,1] = (N_coord[pairs[pair_index][0]][1] + N_coord[pairs[pair_index][1]][1])/2
    for point_index in range(len(pair_index_list), len(pair_index_list)+len(total_point_coverage_points)):
        point_match[point_index] = total_point_coverage_points[point_index-len(pair_index_list)]
        N_coord_pair_center[point_index, 0] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][0]
        N_coord_pair_center[point_index, 1] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][1]
        point_index_list.append(point_index)
    distance_matrix = np.zeros((len(N_coord), len(N_coord)))
    for i in pair_index_list+point_index_list:
        for j in pair_index_list+point_index_list:
            distance_matrix[i,j] = np.linalg.norm(N_coord_pair_center[i]-N_coord_pair_center[j])
    start_node = len(total_area_line_coverage_points+total_point_coverage_points)
    start_node_pair = {}
    start_node_pair[start_node] = route[0]
    for i in pair_index_list+point_index_list:
        distance_matrix[start_node,i] = np.linalg.norm(N_coord[start_node]-N_coord_pair_center[i])
    end_node = route[-1]
    for i in point_index_list+pair_index_list:
        distance_matrix[i,end_node] = np.linalg.norm(N_coord[end_node]-N_coord_pair_center[i])
    # tij_d类比为距离
    new_route_with_pair_index = [start_node]
    unvisited = True
    current_node = start_node
    while unvisited:
        if len(new_route_with_pair_index) == len(pair_index_list + point_index_list)+1:
            unvisited = False
            break
        shortest_distance = np.inf
        # shortest_pair_point = None
        for pair_point_index in pair_index_list + point_index_list:
            if pair_point_index in new_route_with_pair_index:
                continue
            current_distance = distance_matrix[current_node, pair_point_index]
            if current_distance < shortest_distance:
                shortest_distance = current_distance
                shortest_pair_point = pair_point_index
        new_route_with_pair_index.append(shortest_pair_point)
        # print(shortest_pair_point)
        current_node = shortest_pair_point
    # print(new_route_with_pair_index)
    # convert the pair number to the exact node(each pair number is corresponding to 2 nodes)
    exact_node_route = []
    current_node = route[0]
    exact_node_route.append(current_node)
    for pair_index in new_route_with_pair_index:
        if pair_index in pair_index_list:
            if np.linalg.norm(N_coord[current_node]-N_coord[pairs[pair_index][0]]) < np.linalg.norm(N_coord[current_node]-N_coord[pairs[pair_index][1]]):
                # new_route_with_pair_index.remove(pair_index)
                exact_node_route.append(pairs[pair_index][0])
                exact_node_route.append(pairs[pair_index][1])
                current_node = pairs[pair_index][1]
            else:
                # new_route_with_pair_index.remove(pair_index)
                exact_node_route.append(pairs[pair_index][1])
                exact_node_route.append(pairs[pair_index][0])
                current_node = pairs[pair_index][0]
        elif pair_index in point_index_list:
            # new_route_with_pair_index.remove(pair_index)
            exact_node_route.append(total_point_coverage_points[point_index-len(pair_index_list)])
    exact_node_route.append(route[-1])
    return exact_node_route

def NS3(input_truck_route, input_drone_route, tij_k, tij_d, V, inspection_times, is_stop, N_s, S, N_coord):
    truck_route = deepcopy(input_truck_route)
    drone_route = deepcopy(input_drone_route)
    total_area_line_coverage_points = {}
    total_point_coverage_points = {}
    for drone_id, routes in drone_route.items():
        total_area_line_coverage_points[drone_id] = {}
        total_point_coverage_points[drone_id] = {}
    
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            line = []
            point = []
            for s in S:
                for node in N_s[s]:
                    # 处理每个节点
                    if is_stop[s] == 0 and node in route:
                        line.append(node)
                    elif is_stop[s] == 1 and node in route:
                        point.append(node)
            total_area_line_coverage_points[drone_id][sortie_id] = line
            total_point_coverage_points[drone_id][sortie_id] = point
    # print(total_area_line_coverage_points)
    # print(total_point_coverage_points)
    # 遍历每个无人机的覆盖任务
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            # 计算当前路径的总时间
            current_total_time = get_drone_route_time(route, tij_d, 0, inspection_times)['route_time']
            # 按照贪婪算法重新排序覆盖点
            new_route = greedy_reorder(N_coord, route, total_area_line_coverage_points[drone_id][sortie_id], total_point_coverage_points[drone_id][sortie_id])
            # 计算新路径的总时间
            new_total_time = get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time']
            # 如果新路径的总时间更短，则更新路径
            if new_total_time < current_total_time:
                drone_route[drone_id][sortie_id] = new_route
    return truck_route, drone_route

# NS4 average the workload
# Choose the most heavy-loaded drone route and reallocate some work load (points need to be covered) to other available drone route.
def NS4(input_truck_route, input_drone_route, tij_k, tij_d, V, inspection_times, is_stop, N_s, S, DD, N_coord):
    
    truck_route = deepcopy(input_truck_route)
    drone_route = deepcopy(input_drone_route)
    # Calculate the workload of each drone route
    drone_workload = {}
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            drone_workload[(drone_id, sortie_id)] = get_drone_route_time(route, tij_d, 0, inspection_times)['route_time']

    # Find the most heavy-loaded drone route
    max_workload = max(drone_workload.values())
    heavy_loaded_route = [k for k, v in drone_workload.items() if v == max_workload][0]

    # get the line pairs and point coverage points of the heavy-loaded drone route
    total_area_line_coverage_points = []
    total_point_coverage_points = []
    
    for s in S:
        for node in N_s[s]:
            if is_stop[s] == 0 and node in drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]]:
                total_area_line_coverage_points.append(node)
            elif is_stop[s] == 1 and node in drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]]:
                total_point_coverage_points.append(node)
    pairs = {}
    point_match = {}
    pair_index_list = []
    point_index_list = []
    pair_index = 0
    # print(total_area_line_coverage_points)
    # print(total_point_coverage_points)
    
    for i in range(0, len(total_area_line_coverage_points), 2):
        pairs[pair_index] = [total_area_line_coverage_points[i], total_area_line_coverage_points[i+1]]
        pair_index_list.append(pair_index)
        pair_index += 1
    # print(pairs)
    N_coord_pair_center = np.zeros([len(pair_index_list)+len(total_point_coverage_points), 2])
    for pair_index in pair_index_list:
        N_coord_pair_center[pair_index,0] = (N_coord[pairs[pair_index][0]][0] + N_coord[pairs[pair_index][1]][0])/2
        N_coord_pair_center[pair_index,1] = (N_coord[pairs[pair_index][0]][1] + N_coord[pairs[pair_index][1]][1])/2
    for point_index in range(len(pair_index_list), len(pair_index_list)+len(total_point_coverage_points)):
        point_match[point_index] = total_point_coverage_points[point_index-len(pair_index_list)]
        N_coord_pair_center[point_index, 0] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][0]
        N_coord_pair_center[point_index, 1] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][1]
        point_index_list.append(point_index)
        
    distance_matrix = np.zeros((len(N_coord), len(N_coord)))
    for i in pair_index_list+point_index_list:
        for j in pair_index_list+point_index_list:
            distance_matrix[i,j] = np.linalg.norm(N_coord_pair_center[i]-N_coord_pair_center[j])
    start_node = len(total_area_line_coverage_points+total_point_coverage_points)
    start_node_pair = {}
    start_node_pair[start_node] = route[0]
    for i in pair_index_list+point_index_list:
        distance_matrix[start_node,i] = np.linalg.norm(N_coord[start_node]-N_coord_pair_center[i])
    end_node = len(total_area_line_coverage_points+total_point_coverage_points)+1
    end_node_pair = {}
    end_node_pair[end_node] = route[-1]
    for i in point_index_list+pair_index_list:
        distance_matrix[i,end_node] = np.linalg.norm(N_coord[end_node]-N_coord_pair_center[i])
    
    # 找到距离起飞点最远的pair
    farthest_pair = None
    max_distance = 0
    total_index = pair_index_list + point_index_list
    for i in total_index:
        distance = distance_matrix[start_node,i]
        if distance > max_distance:
            max_distance = distance
            farthest_pair = i
    
    if farthest_pair in pair_index_list:
        heavy_loaded_coverage_points = pairs[farthest_pair]
        # drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(heavy_loaded_coverage_points[0])
        # drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(heavy_loaded_coverage_points[1])
        
    elif farthest_pair in point_index_list:
        heavy_loaded_coverage_points = [point_match[farthest_pair]]
        # drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(heavy_loaded_coverage_points[0])

    # Reallocate some workload to other available drone routes
    # 计算每个route的中心
    route_centers = {}
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            route_centers[(drone_id, sortie_id)] = np.mean(N_coord[route], axis=0)

    # 寻找离重载点最近的其他航线（考虑中心距离）
    # 升序搜索并且只需要key不需要value
    route_sort_by_distance = sorted(route_centers.keys(), key=lambda x: np.linalg.norm(route_centers[x] - np.mean(N_coord[heavy_loaded_coverage_points]), axis=0), reverse=False)[1:]
    # shortest_distance = float('inf')
    # for (drone_id, sortie_id), center in route_centers.items():
    #     if (drone_id, sortie_id) != heavy_loaded_route:
    #         distance = np.linalg.norm(center - N_coord[heavy_loaded_coverage_points])
    #         if distance < shortest_distance:
    #             shortest_distance = distance
    #             # 记录下最近的航线
    #             closest_route = (drone_id, sortie_id)

    shortest_time = np.inf
    shortest_route = None
    no_ans = False
    # 将重载点分配给最近的航线,插入点是使得该route飞行时间最短的插入方式
    route_index = 0
    while shortest_time > DD:
        # print(route_sort_by_distance)
        if route_index == len(route_sort_by_distance):
            no_ans = True
            break
        closest_route = route_sort_by_distance[route_index]
        route_index += 1
        shortest_time = np.inf
        shortest_route = None
        no_ans = False

        inner_total_line_coverage_points = []
        inner_total_point_coverage_points = []

        for node in drone_route[closest_route[0]][closest_route[1]]:
            for s in S:
                if is_stop[s] == 0 and node in N_s[s]:
                    inner_total_line_coverage_points.append(node)
                elif is_stop[s] == 1 and node in N_s[s]:
                    inner_total_point_coverage_points.append(node)

        inner_pairs = {}
        inner_point_match = {}
        inner_pair_index_list = []
        inner_point_index_list = []
        inner_pair_index = 0
        for i in range(0, len(inner_total_line_coverage_points), 2):
            inner_pairs[inner_pair_index] = [inner_total_line_coverage_points[i], inner_total_line_coverage_points[i+1]]
            inner_pair_index_list.append(inner_pair_index)
            inner_pair_index += 1
        for point_index in range(len(inner_pair_index_list), len(inner_pair_index_list)+len(inner_total_point_coverage_points)):
            inner_point_match[point_index] = inner_total_point_coverage_points[point_index-len(inner_pair_index_list)]
            inner_point_index_list.append(point_index)
        # 计算pair或point的中心
        inner_N_coord_pair_center = np.zeros([len(inner_pair_index_list)+len(inner_total_point_coverage_points), 2])
        for pair_index in inner_pair_index_list:
            inner_N_coord_pair_center[pair_index,0] = (N_coord[inner_pairs[pair_index][0]][0] + N_coord[inner_pairs[pair_index][1]][0])/2
            inner_N_coord_pair_center[pair_index,1] = (N_coord[inner_pairs[pair_index][0]][1] + N_coord[inner_pairs[pair_index][1]][1])/2
        for point_index in range(len(inner_pair_index_list), len(inner_pair_index_list)+len(inner_total_point_coverage_points)):
            inner_N_coord_pair_center[point_index, 0] = N_coord[inner_total_point_coverage_points[point_index-len(inner_pair_index_list)]][0]
            inner_N_coord_pair_center[point_index, 1] = N_coord[inner_total_point_coverage_points[point_index-len(inner_pair_index_list)]][1]
            # inner_point_index_list.append(point_index)
        can_split = [1]*(len(drone_route[closest_route[0]][closest_route[1]])+1)
        for insert_index in range(1, len(drone_route[closest_route[0]][closest_route[1]])-1):
            for pair_point_index in inner_pair_index_list + inner_point_index_list:
                if pair_point_index in inner_pair_index_list:
                    if drone_route[closest_route[0]][closest_route[1]][insert_index] == inner_pairs[pair_point_index][1]:
                        can_split[insert_index] = 0
        # print(inner_total_line_coverage_points)
        # print(can_split) 
        for insert_index in range(1, len(drone_route[closest_route[0]][closest_route[1]])-1):
            if can_split[insert_index] == 1:
                new_route = drone_route[closest_route[0]][closest_route[1]][:insert_index] + heavy_loaded_coverage_points + drone_route[closest_route[0]][closest_route[1]][insert_index:]
                # print(new_route)
                current_time = get_drone_route_time(new_route, tij_d, 0, inspection_times)['route_time']
                if current_time < shortest_time:
                    shortest_time = current_time
                    shortest_route = new_route
                
    if no_ans == False:
        drone_route[closest_route[0]][closest_route[1]] = shortest_route
        total_index.remove(farthest_pair)
        if farthest_pair in pair_index_list:
            drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(heavy_loaded_coverage_points[0])
            drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(heavy_loaded_coverage_points[1])
        elif farthest_pair in point_index_list:
            drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(heavy_loaded_coverage_points[0])

    return truck_route, drone_route

def NS5(input_drone_route, input_truck_route, S, N_s, tij_d, inspection_times, DD):
    # 如果是同一个区域里的点的sortie可以合并
    drone_route = deepcopy(input_drone_route)
    truck_route = deepcopy(input_truck_route)
    same_area_sortie = {}
    for s in S:
        same_area_sortie[s] = []
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            if all(node in N_s[s] for node in route[1:-1]):
                for s in S:
                    if all(node in N_s[s] for node in route[1:-1]):
                        same_area_sortie[s].append((drone_id, sortie_id))
    for s in S:
        if len(same_area_sortie[s]) > 1:
            base_sortie = same_area_sortie[s][0]
            for other_sortie in same_area_sortie[s][1:]:
                # 合并other_sortie到base_sortie
                # print("Merging sorties:", base_sortie, other_sortie)
                # print(drone_route[base_sortie[0]][base_sortie[1]])
                # print(drone_route[other_sortie[0]][other_sortie[1]][1:-1])
                # print(drone_route[base_sortie[0]][base_sortie[1]][-1])
                new_route_copy = drone_route[base_sortie[0]][base_sortie[1]][:-1] + drone_route[other_sortie[0]][other_sortie[1]][1:-1] + [drone_route[base_sortie[0]][base_sortie[1]][-1]]
                if get_drone_route_time(new_route_copy, tij_d, 0, inspection_times)['route_time'] <= DD:
                    new_route = deepcopy(new_route_copy)
                    drone_route[base_sortie[0]][base_sortie[1]] = new_route
                    del drone_route[other_sortie[0]][other_sortie[1]]
                    if len(drone_route[other_sortie[0]]) == 0:
                        del drone_route[other_sortie[0]]
    return truck_route, drone_route


# NS6随机更换truckroute里的点
def NS6(input_truck_route, input_drone_route, tij_k, tij_d, V, inspection_times, LTD, RTD, TD, N_p):
    truck_route = deepcopy(input_truck_route)
    drone_route = deepcopy(input_drone_route)
    # print(truck_route)
    # 随机挑选0.1*总数个访问点
    num_points_changed = int(0.4 * len(truck_route))
    points_to_change = random.sample(truck_route[1:-1], num_points_changed)
    # print(points_to_change)
    # unchosen_points = [point for point in N_p if point not in truck_route]
    for node in points_to_change:
        if node == truck_route[0] or node == truck_route[-1]:
            continue
        launch_retrieve_dict = {}
        launch_retrieve_dict[node] = set()
        for drone_id, routes in drone_route.items():
            for sortie_id, route in routes.items():
                if node == route[0]:
                    launch_retrieve_dict[node].add(route[-1])
        retrieve_indexes = []
        for retrieve_node in launch_retrieve_dict[node]:
            retrieve_indexes.append(truck_route.index(retrieve_node))
        node_index = truck_route.index(node)
        after_sortie_indexes = []
        for drone_id, routes in drone_route.items():
            for sortie_id, route in routes.items():
                sortie_begin_index = truck_route.index(route[0])
                if sortie_begin_index >= node_index:
                    after_sortie_indexes.append(sortie_begin_index)
        smallest_retrieve_index = min(retrieve_indexes+after_sortie_indexes) if len(retrieve_indexes+after_sortie_indexes) > 0 else len(truck_route)-1
        
        retrieve_launch_dict = {}
        retrieve_launch_dict[node] = set()
        for drone_id, routes in drone_route.items():
            for sortie_id, route in routes.items():
                if node == route[-1]:
                    retrieve_launch_dict[node].add(route[0])
        launch_indexes = []
        for launch_node in retrieve_launch_dict[node]:
            launch_indexes.append(truck_route.index(launch_node))
        before_sortie_indexes = []
        for drone_id, routes in drone_route.items():
            for sortie_id, route in routes.items():
                sortie_end_index = truck_route.index(route[-1])
                if sortie_end_index <= node_index:
                    before_sortie_indexes.append(sortie_end_index)
        largest_launch_index = max(launch_indexes+before_sortie_indexes) if len(launch_indexes+before_sortie_indexes) > 0 else 0
        potential_replacements = []
        if smallest_retrieve_index in retrieve_indexes:
            if largest_launch_index in launch_indexes:
                potential_replacements = [point for point in truck_route if (truck_route.index(point) < smallest_retrieve_index and truck_route.index(point) > largest_launch_index)] + [point for point in N_p if point not in truck_route]
            else:
                potential_replacements = [point for point in truck_route if (truck_route.index(point) < smallest_retrieve_index and truck_route.index(point) >= largest_launch_index)] + [point for point in N_p if point not in truck_route]
        elif smallest_retrieve_index in after_sortie_indexes:
            if largest_launch_index in launch_indexes:
                potential_replacements = [point for point in truck_route if (truck_route.index(point) <= smallest_retrieve_index and truck_route.index(point) > largest_launch_index)] + [point for point in N_p if point not in truck_route]
            else:
                potential_replacements = [point for point in truck_route if (truck_route.index(point) <= smallest_retrieve_index and truck_route.index(point) >= largest_launch_index)] + [point for point in N_p if point not in truck_route]
        # print(potential_replacements)
        # print(largest_launch_index, smallest_retrieve_index)
        potential_replacements_copy = []
        for potential_node in potential_replacements:
            if potential_node == node:
                continue
            if potential_node in points_to_change:
                continue
            potential_replacements_copy.append(potential_node)
        potential_replacements = potential_replacements_copy
        # 随机选择一个替换点
        # print(node)
        # print(potential_replacements)
        # print(potential_replacements)
        if potential_replacements:
            new_point = random.choice(potential_replacements)
            # print(new_point)
            if new_point in truck_route:
                # 删除当前node
                truck_route.remove(node)
            else:
                truck_route[truck_route.index(node)] = new_point
            for drone_id, routes in drone_route.items():
                for sortie_id, route in routes.items():
                    if node == route[0]:
                        route[0] = new_point
                    if node == route[-1]:
                        route[-1] = new_point
        # print(truck_route)
    total_time = get_total_time(drone_route, truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)[0][truck_route[-1]]
    # print('total_time', total_time)
    if total_time > TD:
        truck_route = deepcopy(input_truck_route)
        drone_route = deepcopy(input_drone_route)
    return truck_route, drone_route

def NS7(input_truck_route, input_drone_route, tij_k, tij_d, V, inspection_times, is_stop, N_s, S, DD, N_coord):
    
    truck_route = deepcopy(input_truck_route)
    drone_route = deepcopy(input_drone_route)
    # Calculate the workload of each drone route
    drone_workload = {}
    for drone_id, routes in drone_route.items():
        for sortie_id, route in routes.items():
            drone_workload[(drone_id, sortie_id)] = get_drone_route_time(route, tij_d, 0, inspection_times)['route_time']

    # Find the most heavy-loaded drone route
    max_workload = max(drone_workload.values())
    heavy_loaded_route = [k for k, v in drone_workload.items() if v == max_workload][0]

    # get the line pairs and point coverage points of the heavy-loaded drone route
    total_area_line_coverage_points = []
    total_point_coverage_points = []
    
    for s in S:
        for node in N_s[s]:
            if is_stop[s] == 0 and node in drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]]:
                total_area_line_coverage_points.append(node)
            elif is_stop[s] == 1 and node in drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]]:
                total_point_coverage_points.append(node)
    pairs = {}
    point_match = {}
    pair_index_list = []
    point_index_list = []
    pair_index = 0
    # print(total_area_line_coverage_points)
    # print(total_point_coverage_points)
    
    for i in range(0, len(total_area_line_coverage_points), 2):
        pairs[pair_index] = [total_area_line_coverage_points[i], total_area_line_coverage_points[i+1]]
        pair_index_list.append(pair_index)
        pair_index += 1
    # print(pairs)
    N_coord_pair_center = np.zeros([len(pair_index_list)+len(total_point_coverage_points), 2])
    for pair_index in pair_index_list:
        N_coord_pair_center[pair_index,0] = (N_coord[pairs[pair_index][0]][0] + N_coord[pairs[pair_index][1]][0])/2
        N_coord_pair_center[pair_index,1] = (N_coord[pairs[pair_index][0]][1] + N_coord[pairs[pair_index][1]][1])/2
    for point_index in range(len(pair_index_list), len(pair_index_list)+len(total_point_coverage_points)):
        point_match[point_index] = total_point_coverage_points[point_index-len(pair_index_list)]
        N_coord_pair_center[point_index, 0] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][0]
        N_coord_pair_center[point_index, 1] = N_coord[total_point_coverage_points[point_index-len(pair_index_list)]][1]
        point_index_list.append(point_index)
        
    distance_matrix = np.zeros((len(N_coord), len(N_coord)))
    for i in pair_index_list+point_index_list:
        for j in pair_index_list+point_index_list:
            distance_matrix[i,j] = np.linalg.norm(N_coord_pair_center[i]-N_coord_pair_center[j])
    start_node = len(total_area_line_coverage_points+total_point_coverage_points)
    start_node_pair = {}
    start_node_pair[start_node] = route[0]
    for i in pair_index_list+point_index_list:
        distance_matrix[start_node,i] = np.linalg.norm(N_coord[start_node]-N_coord_pair_center[i])
    end_node = len(total_area_line_coverage_points+total_point_coverage_points)+1
    end_node_pair = {}
    end_node_pair[end_node] = route[-1]
    for i in point_index_list+pair_index_list:
        distance_matrix[i,end_node] = np.linalg.norm(N_coord[end_node]-N_coord_pair_center[i])
    
    # 记录heavy_loaded_route的起飞点和降落点
    heavy_route_launch = drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]][0]
    heavy_route_retrieve = drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]][-1]
    
    # 寻找heavy_loaded_route进行时，没有进行任务的无人机
    idle_drones = []
    heavy_start_truck_route_index = truck_route.index(heavy_route_launch)
    heavy_end_truck_route_index = truck_route.index(heavy_route_retrieve)
    for drone_id, routes in drone_route.items():
        if drone_id != heavy_loaded_route[0]:
            drone_available = True
            # if drone_id != heavy_route_launch:
            for route in routes.values():
                launch_index = truck_route.index(route[0])
                retrieve_index = truck_route.index(route[-1])
                if (launch_index < heavy_start_truck_route_index and retrieve_index > heavy_start_truck_route_index) or (launch_index < heavy_end_truck_route_index and retrieve_index > heavy_end_truck_route_index) or (launch_index >= heavy_start_truck_route_index and retrieve_index <= heavy_end_truck_route_index):
                    drone_available = False
                    break
            if drone_available:
                idle_drones.append(drone_id)
    # print(idle_drones)
                
    # 给heavy_loaded_route的pair进行聚类
    if len(idle_drones) == 0:
        return truck_route, drone_route
    k = min(len(idle_drones)+1, len(pair_index_list)+len(point_index_list))
    kmeans = KMeans(n_clusters=k, random_state=0).fit(N_coord_pair_center)
    labels = kmeans.labels_
    # 根据labels将pair和point进行分类
    cluster_dict = {}
    for idx, label in enumerate(labels):
        if label not in cluster_dict:
            cluster_dict[label] = []
        cluster_dict[label].append(idx)
    # print(cluster_dict)
    # print(pairs[0])
    # 分配给其余空闲无人机（留一组无人机给自己）
    for label in cluster_dict.keys():
        # if label == 0:
        #     continue
        if len(cluster_dict[label]) == 0:
            continue
        if label == labels[0]:
            continue
        assigned_drone = idle_drones.pop(0)
        new_route = [heavy_route_launch]
        for index in cluster_dict[label]:
            if index in pair_index_list:
                new_route.append(pairs[index][0])
                new_route.append(pairs[index][1])
            elif index in point_index_list:
                new_route.append(point_match[index])
        new_route.append(heavy_route_retrieve)
        # drone_route[assigned_drone] = {}
        drone_route[assigned_drone][len(drone_route[assigned_drone].keys())+1] = new_route
        # print(assigned_drone)
        # print(new_route)
        # print(len(drone_route[assigned_drone].keys()))
        # 从heavy_loaded_route中删除这些点
        for index in cluster_dict[label]:
            if index in pair_index_list:
                drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(pairs[index][0])
                drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(pairs[index][1])
            elif index in point_index_list:
                drone_route[heavy_loaded_route[0]][heavy_loaded_route[1]].remove(point_match[index])
    return truck_route, drone_route

def NS8(input_drone_route, input_truck_route, LTD, RTD, tij_k, tij_d, V, inspection_times, N3):
    drone_route = deepcopy(input_drone_route)
    truck_route = deepcopy(input_truck_route)
    # 寻找使得单个sortie的时间减少的最多的起飞点和降落点
    
    # drone_id = random.choice(list(drone_route.keys()))
    # routes = drone_route[drone_id]
    # sortie_id = random.choice(list(routes.keys()))
    # route = routes[sortie_id]
    # 随机选取一个无人机以及其sortie
    drone_id = random.choice(list(drone_route.keys()))
    routes = drone_route[drone_id]
    sortie_id = random.choice(list(routes.keys()))
    route = routes[sortie_id]

    potential_launch_nodes = []
    potential_retrieve_nodes = []
    launch_node = route[0]
    retrieve_node = route[-1]
    shortest_period = get_total_time(drone_route, truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)[0][truck_route[-1]]
    # print(shortest_period)
    launch_index = truck_route.index(launch_node)
    retrieve_index = truck_route.index(retrieve_node)
    unchosen_nodes = []
    previous_sortie_id = None
    subsequent_sortie_id = None
    
    # 遍历该飞机的其余sortie，寻找无人机前一次sortie的降落点并得到index
    smallest_index_difference = np.inf
    for current_sortie_id, current_route in routes.items():
        if current_sortie_id == sortie_id:
            continue
        current_retrieve_index = truck_route.index(current_route[-1]) 
        if launch_index - current_retrieve_index > 0 and launch_index - current_retrieve_index < smallest_index_difference:
            smallest_index_difference = launch_index - current_retrieve_index
            previous_sortie_id = current_sortie_id
        elif launch_index - current_retrieve_index == 0 and launch_index - current_retrieve_index < smallest_index_difference:
            smallest_index_difference = launch_index - current_retrieve_index
            previous_sortie_id = current_sortie_id
            
    # 遍历该飞机的其余sortie，寻找无人机后一次sortie的起飞点并得到index
    smallest_index_difference = np.inf
    for current_sortie_id, current_route in routes.items():
        if current_sortie_id == sortie_id:
            continue
        current_launch_index = truck_route.index(current_route[0]) 
        # if sortie_id == 2 and drone_id == 2:
            # print('current_launch_index', current_launch_index, 'retrieve_index', retrieve_index, 'truck_route',truck_route)
        if current_launch_index - retrieve_index > 0 and current_launch_index - retrieve_index < smallest_index_difference:
            smallest_index_difference = current_launch_index - retrieve_index
            subsequent_sortie_id = current_sortie_id
        elif current_launch_index - retrieve_index == 0 and current_launch_index - retrieve_index < smallest_index_difference:
            
                
            smallest_index_difference = current_launch_index - retrieve_index
            subsequent_sortie_id = current_sortie_id
    
    # for node in N_p:
    #     if node not in truck_route:
    #         unchosen_nodes.append(node)
    
    if previous_sortie_id is not None:
        launch_search_start_index = truck_route.index(routes[previous_sortie_id][-1])
    else:
        launch_search_start_index = 0
    for node in truck_route[launch_search_start_index:launch_index + 1]:
        if node == launch_node:
            potential_launch_nodes.append(node)
            break
        potential_launch_nodes.append(node)
    potential_launch_nodes = potential_launch_nodes + unchosen_nodes
    
    
    if subsequent_sortie_id is not None:
        retrieve_search_end_index = truck_route.index(routes[subsequent_sortie_id][0])
    else:
        retrieve_search_end_index = len(truck_route) - 1
    for idx, node in enumerate(truck_route[retrieve_index: retrieve_search_end_index + 1]):
        potential_retrieve_nodes.append(node)
    potential_retrieve_nodes = potential_retrieve_nodes + unchosen_nodes
    # print(launch_search_start_index, retrieve_search_end_index)
    # print('drone_id', drone_id, 'sortie_id', sortie_id)
    # print('previous_sortie_id', previous_sortie_id, 'subsequent_sortie_id', subsequent_sortie_id)
    temp_route = deepcopy(route)
    # print('original route', route)
    # print(potential_launch_nodes)
    # print(potential_retrieve_nodes)
    for launch in potential_launch_nodes:
        for retrieve in potential_retrieve_nodes:
            if launch != retrieve:
                temp_route[0] = launch
                temp_route[-1] = retrieve
                temp_drone_route = deepcopy(drone_route)
                temp_drone_route[drone_id][sortie_id] = temp_route
                # 计算当前路径的总时间
                current_time = get_total_time(temp_drone_route, truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)[0][truck_route[-1]]
                if current_time <= shortest_period:
                    # print(current_time)
                    shortest_period = current_time
                    # print(f'The route of drone {drone_id} sortie {sortie_id} is changed from {route} to {[launch]+route[1:-1]+[retrieve]}')
                    route[0] = launch
                    route[-1] = retrieve
                    
    # 统计除了当前route所有sortie的起飞和降落点。
    all_sortie_launch_node = set()
    all_sortie_retrieve_node = set()
    
    for cur_drone_id, cur_routes in drone_route.items():
        for cur_sortie_id, cur_route in cur_routes.items():
            if cur_drone_id == drone_id and cur_sortie_id == sortie_id:
                continue
            all_sortie_launch_node.add(cur_route[0])
            all_sortie_retrieve_node.add(cur_route[-1])
            
    change_index = 0
    # if launch_node == route[0] and retrieve_node == route[-1]:
    #     continue
    # print(truck_route)
    # print(all_sortie_launch_node, all_sortie_retrieve_node, launch_node, route[0])
    
    if launch_node != truck_route[0]:
        if launch_node != route[0]:
            if launch_node in all_sortie_launch_node or launch_node in all_sortie_retrieve_node:
                if route[0] not in truck_route:
                    truck_route.insert(launch_index, route[0])
                    change_index = 1
            else:
                if route[0] not in truck_route:
                    truck_route[launch_index] = route[0]
                    change_index = 0
                else:
                    if launch_node != N3[0]:
                        change_index = -1
                        truck_route.remove(launch_node)
    else:
        if launch_node != route[0]:
            if route[0] not in truck_route:
                truck_route.insert(1, route[0])
                change_index = 1
    # else:
    #     print('no change in launch node')
    
    # print(all_sortie_launch_node, all_sortie_retrieve_node, retrieve_node, route[-1])
    if retrieve_node != truck_route[-1]:
        if retrieve_node != route[-1]:
            if retrieve_node in all_sortie_launch_node or retrieve_node in all_sortie_retrieve_node:
                if route[-1] not in truck_route:
                    # print(route[-1] in truck_route)
                    truck_route.insert(retrieve_index+change_index, route[-1])
            else:
                if route[-1] not in truck_route:
                    truck_route[retrieve_index+change_index] = route[-1]
                else:
                    if retrieve_node != N3[-1]:
                        truck_route.remove(retrieve_node)
    else:
        if retrieve_node != route[-1]:
            if route[-1] not in truck_route:
                truck_route.insert(-1, route[-1])
    return truck_route, drone_route

def local_search_method(truck_route, drone_route, tij_k, tij_d, V, inspection_times, local_iter, is_point, N_s, S, DD, p, temperature, global_optimal_time, global_optimal_truck_route, global_optimal_drone_route, N_coord, LTD, RTD, TD, N_p, N3, update_frequency, temperature_decay=0.97):
    local_search_used_count = {'NS1':0, 'NS2':0, 'NS3':0, 'NS5':0, 'NS6':0, 'NS7':0, 'NS8':0}
    local_search_score = {'NS1':0, 'NS2':0, 'NS3':0,'NS5':0, 'NS6':0, 'NS7':0, 'NS8':0}
    local_search_weight = {'NS1':1, 'NS2':1, 'NS3':1, 'NS5':1, 'NS6':1, 'NS7':1,   'NS8':1}
    local_optimal_time = get_total_time(drone_route, truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)[0][truck_route[-1]]
    local_optimal_truck_route = deepcopy(truck_route)
    local_optimal_drone_route = deepcopy(drone_route)
    local_search_lists = ['NS1', 'NS2', 'NS3', 'NS5','NS6', 'NS7', 'NS8']
    # Start local search using NS1-8
    iter_num = 0
    while iter_num <= local_iter:
        # 根据local_search_weight使用轮盘赌选择local search
        local_search = random.choices(local_search_lists, weights=list(local_search_weight.values()))[0]
        
        if local_search == 'NS1':
            new_truck_route, new_drone_route = NS1(truck_route, drone_route, tij_k, tij_d, V, inspection_times, LTD, RTD, DD)

        elif local_search == 'NS2':
            new_truck_route, new_drone_route = NS2(truck_route, drone_route, tij_d, tij_k, inspection_times, DD, V)

        elif local_search == 'NS3':
            new_truck_route, new_drone_route = NS3(truck_route, drone_route, tij_k, tij_d, V, inspection_times, is_point, N_s, S, N_coord)
            
        elif local_search == 'NS4':
            continue
            new_truck_route, new_drone_route = NS4(truck_route, drone_route, tij_k, tij_d, V, inspection_times, is_point, N_s, S, DD, N_coord)

        elif local_search == 'NS5':
            # print("Before", local_search)
            # print(truck_route, drone_route)
            new_truck_route, new_drone_route = NS5(drone_route, truck_route, S, N_s, tij_d, inspection_times, DD)
            new_truck_route, new_drone_route = NS3(new_truck_route, new_drone_route, tij_k, tij_d, V, inspection_times, is_point, N_s, S, N_coord)
            # print("After", local_search)
            # print(new_truck_route, new_drone_route)

        elif local_search == 'NS6':
            new_truck_route, new_drone_route = NS6(truck_route, drone_route, tij_k, tij_d, V, inspection_times, LTD, RTD, TD, N_p)
            
        elif local_search == 'NS7':
            new_truck_route, new_drone_route = NS7(truck_route, drone_route, tij_k, tij_d, V, inspection_times, is_point, N_s, S, DD, N_coord)
        elif local_search == 'NS8':
            new_truck_route, new_drone_route = NS8(drone_route, truck_route, LTD, RTD, tij_k, tij_d, V, inspection_times, N3)
        
        # print("After", local_search)
        # print(new_truck_route, new_drone_route)
        new_total_time = get_total_time(new_drone_route, new_truck_route, tij_k, tij_d, V, inspection_times, LTD, RTD)[0][new_truck_route[-1]]
        local_search_used_count[local_search] += 1
        
        # 去除只有起点和终点的sortie(length=2)
        for drone_id, routes in new_drone_route.items():
            sorties_to_remove = []
            for sortie_id, route in routes.items():
                if len(route) <= 2:
                    sorties_to_remove.append(sortie_id)
                    for node in route:
                        to_remove_from_truck = True
                        for drone_id_inner, routes_inner in new_drone_route.items():
                            for sortie_id_inner, route_inner in routes_inner.items():
                                if node in route_inner and (drone_id_inner, sortie_id_inner) != (drone_id, sortie_id):
                                    to_remove_from_truck = False
                        if to_remove_from_truck and node != new_truck_route[0] and node != new_truck_route[-1]:
                            new_truck_route.remove(node)
            for sortie_id in sorties_to_remove:
                del new_drone_route[drone_id][sortie_id]
        # print("Before", local_search)
        # print(truck_route)
        # print(drone_route)
        # print(f'after {local_search}')
        # print(new_drone_route)
        # print(new_truck_route)
        # 判断生成解是否可以被接受，并处理各个local search的得分
        if new_total_time < global_optimal_time:
            truck_route = deepcopy(new_truck_route)
            drone_route = deepcopy(new_drone_route)
            global_optimal_time = deepcopy(new_total_time)
            local_optimal_time = deepcopy(new_total_time)
            local_optimal_truck_route = deepcopy(new_truck_route)
            local_optimal_drone_route = deepcopy(new_drone_route)
            global_optimal_truck_route = deepcopy(new_truck_route)
            global_optimal_drone_route = deepcopy(new_drone_route)
            local_search_score[local_search] += 30
            
        elif new_total_time < local_optimal_time:
            local_optimal_time = deepcopy(new_total_time)
            local_optimal_truck_route = deepcopy(new_truck_route)
            local_optimal_drone_route = deepcopy(new_drone_route)
            truck_route = deepcopy(new_truck_route)
            drone_route = deepcopy(new_drone_route)
            local_search_score[local_search] += 10
            
        else:
            accept_probability = math.exp(-1*(-local_optimal_time + new_total_time)/temperature)
            random_num = random.random()
            local_search_score[local_search] += 6
            if random_num < accept_probability:
                truck_route = deepcopy(new_truck_route)
                drone_route = deepcopy(new_drone_route)

        iter_num += 1
        # print(local_search)

        if iter_num % update_frequency == 0 and iter_num != 0:
            # print(f"  Local search iteration {iter_num}/{local_iter}, current best time: {global_optimal_time}, local optimal time: {local_optimal_time}")
            temperature = temperature * temperature_decay
            for ls in local_search_lists:
                local_search_weight[ls] = (1-p)*local_search_weight[ls] + p*(local_search_score[ls]/local_search_used_count[ls] if local_search_used_count[ls] != 0 else 0)
                local_search_used_count[ls] = 0
                local_search_score[ls] = 0
        
    return local_optimal_truck_route, local_optimal_drone_route, local_optimal_time, global_optimal_truck_route, global_optimal_drone_route, global_optimal_time, local_search_weight

def self_learning_VNS(N_coord, N1, N2, N3, N_s, N_p, V, dij, tij_k, tij_d, S, DD, is_point, inspection_times, max_iter, local_iter, t, p, stop_same_num, LTD, RTD, TD, local_search_update_frequency, temperature_decay, shaking_reset_num):
    # generate initialized solution
    truck_route, drone_route, init_total_time = initialization(N_coord, N1, N2, N3, N_s, N_p, V, dij, tij_k, tij_d, S, DD, is_point, inspection_times, LTD, RTD)
    
    current_iter = 0
    shaking = ['NS1', 'NS2', 'NS6', 'NS7']
    local_search = ['NS1', 'NS2', 'NS3', 'NS5', 'NS6', 'NS7', 'NS8']
    shaking_score = {'NS1':0, 'NS2':0, 'NS6':0, 'NS7':0}
    # local_search_improvement_count = {'NS1':0, 'NS2':0, 'NS3':0, 'NS4':0, 'NS5':0, 'NS6':0}
    # local_search_used_count = {'NS1':0, 'NS2':0, 'NS3':0, 'NS4':0, 'NS5':0, 'NS6':0}
    # local_search_score = {'NS1':0, 'NS2':0, 'NS3':0, 'NS4':0, 'NS5':0, 'NS6':0}
    # local_search_weight = {'NS1':1, 'NS2':1, 'NS3':1, 'NS4':1, 'NS5':1, 'NS6':1}
    global_optimal_time = deepcopy(init_total_time[0][truck_route[-1]])
    global_optimal_truck_route = deepcopy(truck_route)
    global_optimal_drone_route = deepcopy(drone_route)
    local_optimal_truck_route = deepcopy(truck_route)
    local_optimal_drone_route = deepcopy(drone_route)
    same_num = 0
    total_local_search_weight = {'NS1':0, 'NS2':0, 'NS3':0, 'NS5':0, 'NS6':0, 'NS7':0, 'NS8':0}
    count = 0
    shaking_method_first_num = {'NS1':0, 'NS2':0, 'NS6':0, 'NS7':0}
    
    while current_iter <= max_iter:  
        print(f"Iteration {current_iter}/{max_iter}, current best time: {global_optimal_time}")
        current_iter += 1
        truck_route = deepcopy(local_optimal_truck_route)
        drone_route = deepcopy(local_optimal_drone_route)
        if same_num != 0 and same_num%30 == 0:
            print(f'No improvement in {same_num} iterations, reset to global optimal solution.')
            truck_route = deepcopy(global_optimal_truck_route)
            drone_route = deepcopy(global_optimal_drone_route)
        temperature = t
        # init_global_optimal_time = deepcopy(global_optimal_time)
        shaking_list = sorted(shaking_score.keys(), key=lambda x: shaking_score[x], reverse=True)
        shaking_method_first_num[shaking_list[0]] += 1
        chosen = None
        # print("  Before shaking")
        # print(truck_route, drone_route)
        # if current_iter == 0:
        #     shaking = 'NS7'
        before_shaking_local_search_optimal_time = deepcopy(global_optimal_time)
        global_optimal_time_copy = deepcopy(global_optimal_time)
        i=0
        
        while i < len(shaking_list):
            shaking = shaking_list[i]
            if shaking == 'NS1':
                truck_route, drone_route = NS1(truck_route, drone_route, tij_k, tij_d, V, inspection_times, LTD, RTD, DD)
                chosen = 'NS1'
            elif shaking == 'NS2':
                truck_route, drone_route = NS2(truck_route, drone_route, tij_d, tij_k, inspection_times, DD, V)
                chosen = 'NS2'
            elif shaking == 'NS6':
                truck_route, drone_route = NS6(truck_route, drone_route, tij_k, tij_d, V, inspection_times, LTD, RTD, TD, N_p)
                chosen = 'NS6'
            elif shaking == 'NS7':
                truck_route, drone_route = NS7(truck_route, drone_route, tij_k, tij_d, V, inspection_times, is_point, N_s, S, DD, N_coord)
                chosen = 'NS7'
            
            
            # print(f"  After shaking {chosen}")
            # print(truck_route, drone_route)
            local_optimal_truck_route, local_optimal_drone_route, local_optimal_time, global_optimal_truck_route, global_optimal_drone_route, global_optimal_time, local_search_weight = local_search_method(truck_route, drone_route, tij_k, tij_d, V, inspection_times, local_iter, is_point, N_s, S, DD, p, temperature, global_optimal_time, global_optimal_truck_route, global_optimal_drone_route, N_coord, LTD, RTD, TD, N_p, N3, local_search_update_frequency,  temperature_decay)
            count += 1
            for ls in local_search:
                total_local_search_weight[ls] += local_search_weight[ls]
            # print(f"  After local search")
            # print(local_optimal_truck_route, local_optimal_drone_route)
            
            if global_optimal_time < global_optimal_time_copy:
                shaking_score[chosen] += 1
                global_optimal_time_copy = deepcopy(global_optimal_time)
                i = 0
            else:
                i += 1
        
        if global_optimal_time < before_shaking_local_search_optimal_time:
            same_num = 0
        if current_iter % shaking_reset_num == 0 and current_iter != 0:
            for sk in shaking:
                shaking_score[sk] = 0
        if abs(before_shaking_local_search_optimal_time - global_optimal_time) < 1e-6:
            same_num += 1
        if same_num >= stop_same_num:
            print(f"No improvement in {stop_same_num} iterations, stopping early.")
            break
    for ls in local_search:
        total_local_search_weight[ls] = total_local_search_weight[ls]/count
    init_time = init_total_time[0][truck_route[-1]]
    improvement = (init_time - global_optimal_time)/init_time
    iter_num = current_iter
    return global_optimal_truck_route, global_optimal_drone_route, global_optimal_time, total_local_search_weight, shaking_method_first_num, init_time, improvement, iter_num