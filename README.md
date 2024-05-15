# sensor_location
## Modeling Preparation

(1) Basic Elements of the Traffic Network

The basic elements of the traffic network include nodes (intersections) and edges (road segments).

## Detector Layout Optimization Algorithm

### Algorithm 1: Minimum Detector Layout Optimization Algorithm Under Full Path Observability

(1) Function

Given the topology of all paths and road segments, this algorithm provides the minimum number of active (AVI) detectors needed to observe all paths and their positions. For specific model formulas, please refer to the section "7.1 Full Route Observability Problem" in my paper "Route Flow Estimation Based on the Fusion of Probe Vehicle Trajectory and Automated Vehicle Identification Data."

(2) Description

`route_link_indicator` is a 0-1 sparse matrix indicating whether path `r` includes road segment `a` (1 for yes, 0 for no). This is primarily constructed based on the `df_route` and `df_link` dataframes. Although constructing these two input dataframes can be complex, the code can be modified in specific applications to simplify this process.

Code path: `model_param.py`, function `get_indicator`.

(3) Others

Code path: `observability_model.py`, function `full_observability`.

For specific case applications, please refer to `sensor_location_model_flow_observability.ipynb`.

### Algorithm 2: Maximum Observable Path Count Under Given Detector Quantity Constraint

(1) Function

Unlike Algorithm 1, this algorithm maximizes the number of observable paths given a fixed number of detectors. For specific model formulas, please refer to the section "7.2 Budget-Constrained Maximum Route Flow Observability Problem" in my paper "Route Flow Estimation Based on the Fusion of Probe Vehicle Trajectory and Automated Vehicle Identification Data."

(2) Others

Code path: `observability_model.py`, function `partial_observability`.

For specific case applications, please refer to `sensor_location_optimization_eg1.ipynb`.

## Path Flow Estimation Algorithm

### Algorithm 2: Path Flow Estimator

For specific case applications, please refer to `sensor_location_model_PFE.ipynb`.



The following is the Chinese version. 

# sensor_location 
## 建模准备 

（1）交通网络基本要素

交通网络基本要素包括节点（交叉口）、边（路段）

## 检测器布局优化算法

### 算法1：全路径可观测条件下最少检测器布局优化算法

（1）功能

在已知所有路径和路段拓扑结构的前提下，给出所有路径可观测的最少需要主动型（AVI）检测器的数量及其位置。具体模型公式，请参考本人论文“Route Flow Estimation Based on the Fusion of Probe Vehicle Trajectory and Automated Vehicle Identification Data”的`7.1 Full Route Observability Problem`这一章节。

（2）说明

`route_link_indicator`是一个0-1稀疏矩阵，表示路径`r`是包含路段`a`，1表示是，0表示否。其构建方法主要基于`df_route`、`df_link`两个`dataframe`数据开展。虽然构建这两个输入数据的条件有点复杂，但自己的应用中是可以修改代码规避的。

代码路径：`model_param.py`中的`get_indicator`函数。

（3）其它

代码路径：`observability_model.py`内的`full_observability`函数。

具体案例应用，请参考`sensor_location_model_flow_observability.ipynb`。

### 算法2：给定检测器数量约束下的可观测路径数量最大化算法

（1）功能

区别于前面算法1，在给定检测器数量的前提下，尽可能最大化可观测路径数量。具体模型公式，请参考本人论文“Route Flow Estimation Based on the Fusion of Probe Vehicle Trajectory and Automated Vehicle Identification Data”的`7.2 Budget-Constrained Maximum Route Flow Observability Problem`这一章节。

（2）其它

代码路径：`observability_model.py`内的`partial_observability`函数。

具体案例应用，请参考`sensor_location_optimization_eg1.ipynb`。

## 路径流量估计算法

### 算法2：Path Flow Estimator

具体案例应用，请参考`sensor_location_model_PFE.ipynb`。
