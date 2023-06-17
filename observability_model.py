# traffic sensor location optimization model

# import packages
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import scipy.sparse as sp
import pickle


def get_scanned_links(model, link_scan_binary):
    '''get scanned links from the optimized model'''
    if model.status == GRB.OPTIMAL:
        print('Links being scanned:', end=' ')
        link_scanned_list = []
        for link in link_scan_binary:
            if link_scan_binary[link].X > .9:  # this is not a specific integer !!!
                link_scanned_list.append(link)
                print('%d' % link, end=' ')
        print('')
        print('Number of links being scanned:', len(link_scanned_list))
    else:
        print('model do not achieve optimum.')
    return len(link_scanned_list), link_scanned_list


def full_observability(num_route, num_link, route_link_indicator):
    '''
    Minimum number of sensors to achieve full observability.
    :param num_route: number of routes
    :param num_link: number of links
    :param route_link_indicator: route-link incidence matrix
    :return: number of links with sensors, list of links with sensors
    '''
    
    # create a new model
    model = gp.Model("full_observability")

    # create variables
    # variable 1: whether a sensor is installed on a link
    link_scan_binary = model.addVars(num_link, vtype=GRB.BINARY, name='link_scan_binary')

    # objective function
    model.setObjective(link_scan_binary.sum(), GRB.MINIMIZE)

    # constraints
    # constraint 1: whether a route is observable
    route_pair_list = [(route_1, route_2) for route_1 in range(num_route) for route_2 in range(route_1 + 1, num_route)]
    model.addConstrs(sum([link_scan_binary[link] for link in range(num_link) if route_link_indicator[route_1, link] != route_link_indicator[route_2, link]]) >= 1 
                     for route_1, route_2 in route_pair_list)  # at least one link is different
    
    # constraint 2: at least one sensor is installed on a observable route
    model.addConstrs(sum([link_scan_binary[link] for link in range(num_link) if route_link_indicator[route, link] == 1]) >= 1
                        for route in range(num_route))  # at least one sensor is installed on a observable route
    
    # optimize the model
    model.optimize() 
    
    # get the list of links with sensors
    link_scan_binary_list = [link_scan_binary[link].X for link in range(num_link)]
    link_scanned_list = [link for link in range(num_link) if link_scan_binary_list[link] > .9]  # this is not a specific integer !!!

    # get the number of links with sensors
    num_link_scanned = len(link_scanned_list)

    return num_link_scanned, link_scanned_list


def partial_observability(max_num_sensors, num_route, num_link, route_link_indicator):
    '''
    Budget constrained optimal sensor layout formulation
    '''

    # create a new model
    model = gp.Model("partial_observability")

    # create variables
    # variable 1: whether a sensor is installed on a link
    link_scan_binary = model.addVars(num_link, vtype=GRB.BINARY, name='link_scan_binary')

    # variable 2: whether a route is observable
    route_idtf_binary = model.addVars(num_route, vtype=GRB.BINARY, name='route_idtf_binary')

    # objective function: maximize the number of observable routes
    model.setObjective(route_idtf_binary.sum(), GRB.MAXIMIZE)

    # constraints
    # constraint 1: whether a route is observable
    route_pair_list = [(route_1, route_2) for route_1 in range(num_route) for route_2 in range(route_1 + 1, num_route)]
    model.addConstrs(sum([link_scan_binary[link] for link in range(num_link) if route_link_indicator[route_1, link] != route_link_indicator[route_2, link]]) >= route_idtf_binary[route_1]
                        for route_1, route_2 in route_pair_list)  # at least one link is different

    # constraint 3: sensor budget
    model.addConstr(link_scan_binary.sum() <= max_num_sensors)

    # optimize the model
    model.Params.OutputFlag = 0
    model.optimize()

    # get the list of links with sensors
    link_scan_binary_list = [link_scan_binary[link].X for link in range(num_link)]
    link_scanned_list = [link for link in range(num_link) if link_scan_binary_list[link] > .9]  # this is not a specific integer !!!
    # get the number of links with sensors
    num_link_scanned = len(link_scanned_list)

    return num_link_scanned, link_scanned_list
    

    
