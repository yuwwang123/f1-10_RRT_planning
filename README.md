Implementation of real-time RRT* on the ROS-enabled F1/10 autonomous racing car. At each time-step, RRT* plans a path to a goal that is certain distance ahead of current car position. The goal gets advanced along the track centerline every iteration as the car navigates. The read region shown in the occupancy grid map below represents the inflated track boundaries and obstacles.

![](rrt_star.gif)

Testing on F1/10 hardware:

![](car_testing2.gif)

