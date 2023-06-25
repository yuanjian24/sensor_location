import gurobipy as gp
from gurobipy import GRB
# import numpy as np
# import cmath

def solve(sensor_number, 
          sensor_links,  
          sensor_routes,  # PFE-enhanced model
          od_route,
          num_route,
          num_link,
          link_set,
          sensor_link_flow_obs, # 所有带传感器路段的观测流量
          all_link_flow_dict,  # 所有路段的真实流量，可以用于制作 Prior link flow
          route_link_indicator,
          ground_truth_route_flow,
          ground_truth_od_flow,
          theta  # dispersion parameter
    ):
    '''
    Path Flow Estimator Algorithm

    Param
    - sensor_number: the number of AVI sensors
    - sensor_links: the list of scanned links
    - sensor_routes: the list of observable routes
    - od_route: 

    '''
    # Model
    model = gp.Model('path_flow_estimator')

    # Variables
    # Estimated route flow - decision variables f_i,r
    route_flow_est = model.addVars(od_route, name='route_flow_est')
    # route flow help variable
    route_flow_est_ln = model.addVars(od_route, name='route_flow_est_ln')

    # Estimated link flow x_a
    link_list = list(range(num_link))
    link_flow_est = model.addVars(link_list, name='link_flow_est')
    # Link flow help
    link_flow_est_two_fold = model.addVars(link_list, name='link_flow_est_two_fold')
    link_flow_est_four_fold = model.addVars(link_list, name='link_flow_est_four_fold')
    link_flow_est_five_fold = model.addVars(link_list, name='link_flow_est_five_fold')

    # Route-link indicator (binary var, for constraint 1-1)
    route_link_ind_variable = model.addVars(num_route, num_link,
                                            vtype=GRB.BINARY,
                                            name='route_link_ind')

    # Objective function
    expr = gp.QuadExpr()
    # e = float(np.exp(1))

    # theta = 0.01 # dispersion parameter
    for od, route in od_route:
        obj_1_temp = route_flow_est[od, route] * (route_flow_est_ln[od, route] - 1)
        expr +=  1 / theta * obj_1_temp

    alpha, beta = 0.15, 4.00
    for _, each_link in link_set.items():
        link = each_link.link_idx - 1
        fft = each_link.free_flow_time
        capacity  = each_link.capacity
        
        # t_0 * x + t_0 * alpha / (C_a)^beta / beta * x^(beta+1)
        travel_cost_calculus_temp = fft * link_flow_est[link] + \
        fft * alpha / (capacity ** beta) / beta * link_flow_est_five_fold[link]
        expr += travel_cost_calculus_temp

    model.setObjective(expr, GRB.MINIMIZE)

    # Constraints

    # allowed measurement error of the link flow
    epslon = 0.05
    if sensor_number == 14:
        epslon = 0.10
    # C1: Observed link flow
    for link in sensor_links:
        model.addConstr(lhs=link_flow_est[link],
                        sense=GRB.LESS_EQUAL,
                        rhs=sensor_link_flow_obs[link] * (1 + epslon),
                        name='observed_link_flow_lower %s' % link
                        )
        model.addConstr(lhs=link_flow_est[link],
                        sense=GRB.GREATER_EQUAL,
                        rhs=sensor_link_flow_obs[link] * (1 - epslon),
                        name='observed_link_flow_upper %s' % link
                        )

    # C2: Capacity upper bound
    for _, each_link in link_set.items():
        link = each_link.link_idx - 1
        capacity  = each_link.capacity
        model.addConstr(lhs=link_flow_est[link],
                        sense=GRB.LESS_EQUAL,
                        rhs=capacity,
                        name='capacity_bound %s' % link
                        )

    # Innitialize the binary variables
    model.addConstrs((route_link_ind_variable[route, link] ==
                     route_link_indicator[route, link]
                     for route in range(num_route)
                     for link in range(num_link)),
                     name='delta')

    # C3: relationship between route flow and link Flow
    for link in link_list:
        model.addConstr(lhs=link_flow_est[link],
                        sense=GRB.EQUAL,
                        rhs=sum([(route_flow_est[od, route] *
                                  route_link_indicator[route, link])
                                 for (od, route) in od_route]),
                        name='link_flow_est %s' % link
                        )

    # C4: the two_fold, four_fold, five_fold variables
    for link in link_list:
        model.addConstr(lhs=link_flow_est_two_fold[link],
                        sense=GRB.EQUAL,
                        rhs=link_flow_est[link] * link_flow_est[link],
                        name='link_flow_two_fold %s' % link
                        )

    for link in link_list:
        model.addConstr(lhs=link_flow_est_four_fold[link],
                        sense=GRB.EQUAL,
                        rhs=link_flow_est_two_fold[link] * link_flow_est_two_fold[link],
                        name='link_flow_four_fold %s' % link
                        )

    for link in link_list:
        model.addConstr(lhs=link_flow_est_five_fold[link],
                        sense=GRB.EQUAL,
                        rhs=link_flow_est_four_fold[link] * link_flow_est[link],
                        name='link_flow_five_fold %s' % link
                        )
    # C5: Ln Constraint
    for od, route in od_route:
        model.addGenConstrLog(route_flow_est[od, route],
                              route_flow_est_ln[od, route],
                              name='route_flow_ln %s' % route)

    # C6: For those routes that can be observed
    # epslon = 0.05
    # for od, route in od_route:
    #     if route in sensor_routes:
    #         model.addConstr(lhs=route_flow_est[od, route],
    #                         sense=GRB.LESS_EQUAL,
    #                         rhs=ground_truth_route_flow[od, route] * (1 + epslon),
    #                         name='route_observability_upper %s' % route
    #                         )
    #         model.addConstr(lhs=route_flow_est[od, route],
    #                         sense=GRB.GREATER_EQUAL,
    #                         rhs=ground_truth_route_flow[od, route] * (1 - epslon),
    #                         name='route_observability_lower %s' % route
    #                         )

    # C7: The OD flow Constraint
    # od_list = [od for od, route in od_route]
    # epslon_od = 0.20
    # for od in od_list:
    #     route_list_tmp = [route_tmp for od_tmp, route_tmp in od_route if od_tmp == od]
    #     model.addConstr(lhs=ground_truth_od_flow[od],
    #                     sense=GRB.GREATER_EQUAL,
    #                     rhs=sum([route_flow_est[od, route]
    #                             for route in route_list_tmp]) * (1 - epslon_od),
    #                     name='od_flow_lower %s' % od
    #                     )
    #     model.addConstr(lhs=ground_truth_od_flow[od],
    #                     sense=GRB.LESS_EQUAL,
    #                     rhs=sum([route_flow_est[od, route]
    #                             for route in route_list_tmp]) * (1 + epslon_od),
    #                     name='od_flow_upper %s' % od
    #                     )

    # model.Params.OutputFlag = 0
    model.Params.NonConvex = 2
    model.optimize()

    def printSolution():
        if model.status == GRB.OPTIMAL:
            print('Achieve optimal')

            result_route_flow = model.getAttr('x', route_flow_est)

            # RMSE of route flow (only available in simulation data)
            rmse_route_flow = 0
            # MAPE of route flow
            mape_route_flow = 0

            for (od, route), route_flow in ground_truth_route_flow.items():
                rmse_route_flow += (route_flow -
                                    result_route_flow[od, route]
                                    ) ** 2 / len(ground_truth_route_flow)
                if route_flow == 0:
                    continue
                else:
                    mape_route_flow += abs(route_flow - result_route_flow[od, route]) \
                                       / route_flow

            rmse_route_flow = rmse_route_flow ** 0.5
            mape_route_flow /= len(ground_truth_route_flow)
        else:
            print('Not Optimal')
            rmse_route_flow, mape_route_flow, result_route_flow = -1, -1, -1

        return rmse_route_flow, mape_route_flow, result_route_flow #, rmse_link_flow_all, mape_link_flow_all


    rmse_route_flow, mape_route_flow, result_route_flow = printSolution()
    return rmse_route_flow, mape_route_flow, result_route_flow
