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

具体案例应用，请参考`sensor_location_optimization_eg1.ipynb`。

### 算法2：给定检测器数量约束下的可观测路径数量最大化算法

（1）功能

区别于前面算法1，在给定检测器数量的前提下，尽可能最大化可观测路径数量。具体模型公式，请参考本人论文“Route Flow Estimation Based on the Fusion of Probe Vehicle Trajectory and Automated Vehicle Identification Data”的`7.2 Budget-Constrained Maximum Route Flow Observability Problem`这一章节。

（2）其它

代码路径：`observability_model.py`内的`partial_observability`函数。

具体案例应用，请参考`sensor_location_optimization_eg1.ipynb`。

## 路径流量估计算法

### 算法2：Path Flow Estimator

（待写）
