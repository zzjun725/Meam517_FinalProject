## Optimization-based collision-free trajectory generation for quadrotors.

Developed in ROS1, frontend use A* search and cuboid corridors, backend use Bezier curve and min-jerk trajectory optimization. 

**Result**:
The figure shows the same trajectory in three different viewpoints. The map is constructed with point cloud library. The red dots
are A∗ waypoints. The grey cuboids are flight corridors. The blue curve is the final trajectory. Left and right figures are from the same viewpoint where the obstacles in the left figure
are set to transparent so as to show the trajectory more clearly.

**Team**: Qirui Wu, Zhijun Zhuang, Mengti Sun.

**Reference**:

F. Gao, W. Wu, Y. Lin and S. Shen, "Online Safe Trajectory Generation for Quadrotors Using Fast Marching Method and Bernstein Basis Polynomial," 
2018 IEEE International Conference on Robotics and Automation (ICRA), Brisbane, QLD, Australia, 2018, pp. 344-351, doi: 10.1109/ICRA.2018.8462878.
