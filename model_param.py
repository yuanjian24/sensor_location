### Get all the necessary parameters for the model

import pandas as pd
import numpy as np
import scipy.sparse as sp
import scipy.stats as stats
import gurobipy as gp
from gurobipy import GRB
import observability_model as om 
import random
import pickle


def str2list(route_str):
    '''
    将列表的str形式转化为真正的list形式
    '''
    route_list = route_str.lstrip('[').rstrip(']').split(', ') 
    route_list = [int(i) for i in route_list]
    return route_list


def get_indicator(num_route, num_link, df_route, df_link): 
    '''
    Get the route-link indicator.
    如有绕路行为，则该对应数值为路径重复经过该路段的次数。
    '''
    row, col = [], []
    for route_idx in range(num_route): 
        if type(df_route.loc[route_idx, 'route_node']) == str:
            node_list = str2list(df_route.loc[route_idx, 'route_node'])  # 节点列表
        else: 
            node_list = df_route.loc[route_idx, 'route_node']  # 节点列表
            # node_list = df_route.loc[route_idx, 'route_node'] # 调用
        # print(node_list)
        for node_idx in range(len(node_list) - 1):
            link_idx = df_link[(df_link['u']==node_list[node_idx]) & 
                               (df_link['v']==node_list[node_idx + 1])].index[0]
            # print(route_idx, link_idx)
            row.append(route_idx)
            col.append(link_idx)

    time_link_used = len(row) 
    data = np.array([1] * time_link_used)
    
    row = np.array(row)
    col = np.array(col)

    route_link_indicator = sp.csr_matrix((data, (row, col)), shape=(num_route, num_link))
    return route_link_indicator


def get_route_sensor_od_indicator(num_route, 
                                  num_link, 
                                  route_link_indicator,
                                  scanned_links, 
                                  df_route,
                                  df_link
                                 ):
    '''
    思路：
    - 针对每一条路径，若先经过sensor1，再经过sensor2，则该indicator为1，否则为0
    - 即便有绕路重复经过行为，一条路径的流量也仅加载一次
    '''
    route_sensor_od_indicator = np.zeros([num_route, num_link, num_link])
    
    for route in range(num_route):
        # get the link list
        route_link_list = []
        if type(df_route.loc[route, 'route_node']) == str:
            route_node_list = str2list(df_route.loc[route, 'route_node'])  # 节点列表
        else:
            route_node_list = df_route.loc[route, 'route_node']
            
        for node_idx in range(len(route_node_list) - 1):
            node_1, node_2 = route_node_list[node_idx], route_node_list[node_idx + 1]
            link_idx = df_link[(df_link['u']==node_1) & (df_link['v']==node_2)].index[0] 
            route_link_list.append(link_idx) 
        
        sensor_od_pair = [(sensor_1, sensor_2) for sensor_1 in scanned_links for sensor_2 in scanned_links 
                          if sensor_1 != sensor_2]
        for (sensor_1, sensor_2) in sensor_od_pair: 
            if (route_link_indicator[route, sensor_1] > 0) and (route_link_indicator[route, sensor_2] > 0):
                # 这里之前写的是 都等于1，这在有绕路的场景下是wrong的，因为：会导致不一致
                if route_link_list.index(sensor_1) < route_link_list.index(sensor_2):
                    route_sensor_od_indicator[route, sensor_1, sensor_2] = 1
    return route_sensor_od_indicator 


def get_sensor_OD_flow(num_route, num_link, route_sensor_od_indicator, scanned_links, df_route):
    sensor_OD = {} # 存储sensor OD flow
    sensor_od_pair = [(sensor_1, sensor_2) for sensor_1 in scanned_links for sensor_2 in scanned_links 
                          if sensor_1 != sensor_2]
    
    for (sensor_1, sensor_2) in sensor_od_pair:
        sensor_OD_tmp = 0
        for route in range(num_route):
            sensor_OD_tmp += df_route.loc[route, 'route_flow'] * route_sensor_od_indicator[route, sensor_1, sensor_2]
        # 赋值
        sensor_OD[sensor_1, sensor_2] = sensor_OD_tmp
    return sensor_OD 


def route_to_link(node_list, df_link):
    link_list = []
    for node_idx in range(len(node_list) - 1):
        link_id = df_link[(df_link['u']==node_list[node_idx]) & 
                          (df_link['v']==node_list[node_idx + 1])].index[0]
        link_list.append(link_id)
    return link_list 


def get_link_flow(df_link, df_route, route_link_indicator):
    df_link['link_flow'] = 0
    for route in range(len(df_route)):
        route_flow = df_route.loc[route, 'route_flow']  # 对应路径流量
        
        # 获取路段列表
        if type(df_route.loc[route, 'route_node']) == str:
            node_list = str2list(df_route.loc[route, 'route_node'])  # 节点列表
        else: 
            node_list = df_route.loc[route, 'route_node']  # 节点列表
        
        link_list = set(route_to_link(node_list, df_link))  # 路段列表，去重。如有重复，用route-link-indicator来处理即可
        for link in link_list: 
            df_link.loc[link, 'link_flow'] += route_flow * route_link_indicator[route, link]
            # 问题：是否需要乘以 route-link indicator? 答：有绕路的case下，是需要的
    return df_link 


def bpr_link_tt(each_link_flow, each_link_length, error_ratio=0.235):
    '''
    BPR link cost function, with random error
    alpha = 0.15
    beta = 4.00
    '''
    capacity = 300
    f_speed = 10  # m/s
    fft = each_link_length / f_speed
    
    # true link tt
    each_link_tt = fft * (1 + 0.3 * each_link_flow)  # 简化函数

    # 加或减
    plus_or_minus = random.randint(0, 1)
    if plus_or_minus == 0:
        plus_or_minus = -1
    
    # random error for estimation
    # random error term
    random_term = plus_or_minus * each_link_tt * error_ratio * random.random()
    each_link_tt_obs = each_link_tt + random_term
    # each_link_tt = fft * (1 + 0.15 * (each_link_flow ** 2 / capacity))
    return each_link_tt_obs    


# 获取模型参数具体版本
def get_final_param(num_sensor):
    
    # 读取基础数据文件
    pkl_file = open('model_input/df_route.pkl', 'rb') 
    df_route = pickle.load(pkl_file) 
    df_route = df_route.drop(['route_flow_true', 'o_lon_col', 'o_lat_col', 'd_lon_col', 'd_lat_col'], axis=1)
    
    pkl_file = open('model_input/edges_used.pkl', 'rb') 
    df_link = pickle.load(pkl_file) 
    
    # initialization of parameters

    # 1 num_route, num_link, num_sensor
    num_route, num_link, num_sensor = len(df_route), len(df_link), num_sensor
    print('num_route:', num_route, 'num_link:', num_link, 'num_sensor:', num_sensor)
    
    df_route['route_id'] = [i for i in range(num_route)]

    # 2 route_link_indicator
    route_link_indicator = get_indicator(num_route, num_link, df_route, df_link) # 不费时
    # print(route_link_indicator.max())
    # pkl_file = open('model_input/route_link_indicator.pkl', 'rb')
    # route_link_indicator = pickle.load(pkl_file) 

    # 4 scanned_links & observable_routes (read from pickle files)
    pkl_file = open('model_input/exp_design/link_scanned_{}_sensors.pkl'.format(num_sensor), 'rb')
    scanned_links = pickle.load(pkl_file) 

    pkl_file = open('model_input/exp_design/route_identified_{}_sensors.pkl'.format(num_sensor), 'rb')
    observable_routes = pickle.load(pkl_file) 

    sensor_od_pair = [(sensor_1, sensor_2) for sensor_1 in scanned_links for sensor_2 in scanned_links 
                      if sensor_1 != sensor_2]

    # 5 route_sensor_od_indicator
    route_sensor_od_indicator = get_route_sensor_od_indicator(num_route, num_link, 
                                                              route_link_indicator,
                                                              scanned_links, 
                                                              df_route, 
                                                              df_link)
    # pkl_file = open('model_input/exp_design/route_sensor_od_indicator_{}_sensors.pkl'.format(num_sensor), 'rb') 
    # route_sensor_od_indicator = pickle.load(pkl_file) 

    # 6 route flow
    route_flow = list(df_route['route_flow'])
    
    # 7 rpr generation
    # rpr_center, sigma = 0.50, 0.5  # RPR均值与方差 
    # lower, upper = 0, 1  # 截断分布上下限 
    # X = stats.truncnorm((lower - rpr_center) / sigma, (upper - rpr_center) / sigma, loc=rpr_center, scale=sigma)
    # rpr = X.rvs(num_route)  # 路径渗透率列表 
    # print('rpr:', rpr)
    
    # df_route['rpr'] = rpr 
    
    # 8 route_flow_probe
    # df_route['route_flow_probe'] = df_route.apply(lambda r: r['route_flow'] * r['rpr'], axis=1)
    # route_flow_probe = list(df_route['route_flow_probe'])

    # 9 sensor od flow
    sensor_od = get_sensor_OD_flow(num_route, num_link, route_sensor_od_indicator, scanned_links, df_route)
    # for (sensor_1, sensor_2) in sensor_od_pair:
    #     for route in range(num_route):
    #         if route_sensor_od_indicator[route, sensor_1, sensor_2] >= 1: 
    #             print(route, sensor_1, sensor_2, ', sensor OD:', sensor_od[sensor_1, sensor_2])

    # 10 link flow
    df_link = get_link_flow(df_link, df_route, route_link_indicator)
    link_flow = list(df_link['link_flow'])
    
    # 11 link length
    link_length = list(df_link['length'])

    # 12 route travel time
    # 加载出行数据
    # import pandas as pd
    # df_trip = pd.read_csv('../data/pneuma/flow matrix/trip_20181024.csv') 
    # df_trip_2 = df_trip[-(df_trip['o_lon_col']==df_trip['d_lon_col']) & 
    #                      (df_trip['o_lat_col']==df_trip['d_lat_col'])]
    # print('在{}次针出行记录中，总共有{}辆车的起讫点属于不同栅格'.format(len(df_trip), len(df_trip_2)))

    # # 给每个trip附上路径ID
    # df_trip_2 = pd.merge(df_trip_2[['route_node', 'travel_time']], 
    #                      df_route[['route_node', 'route_id']], on='route_node')

    # # 统计每个路径的平均行程时间
    # df_route_tt = df_trip_2.groupby('route_id')['travel_time'].mean().rename('route_tt').reset_index()
    # print(len(df_route_tt))
    # route_tt = list(df_route_tt['route_tt'])

    
    # 12 synthesized travel time
    error_ratio = 0.01 #  
    link_tt = [bpr_link_tt(link_flow[link], link_length[link], error_ratio) for link in range(num_link)] 

    route_tt = [] 
    for route in range(num_route):
        tmp = sum([link_tt[link] * route_link_indicator[route, link] for link in range(num_link)])
        route_tt.append(tmp)
    
    
    # 13 od对的列表
    od_list = list(df_route.drop_duplicates('OD_id')['OD_id'])
    
    # 构建每个OD对与路径编号的关系
    od_route_dict = {}
    for od in od_list:
        od_route_dict[od] = []
    # print(od_route_dict)

    for route in range(num_route):
        od = df_route.loc[route, 'OD_id'] 
        route = df_route.loc[route, 'route_id'] 
        od_list = od_route_dict[od]
        if route not in od_list:
            od_list.append(route)
            od_route_dict[od] = od_list
    # print(od_route_dict)
    
    # return num_route, num_link, scanned_links, observable_routes, od_route_dict, route_link_indicator, \
    #        route_sensor_od_indicator, route_flow, route_flow_probe, link_flow, sensor_od, \
    #        link_length, route_tt, df_route, df_link
    return num_route, num_link, scanned_links, observable_routes, od_route_dict, route_link_indicator, \
           route_sensor_od_indicator, route_flow, link_flow, sensor_od, \
           link_length, route_tt, df_route, df_link
