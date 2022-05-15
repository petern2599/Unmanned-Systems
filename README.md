# Unmanned-Systems
Contains scripts made during the course ME-459

## Path-Planning
[Path-Planning Sub-Directory](https://github.com/petern2599/Unmanned-Systems/tree/main/Path-Planning%20Algorithms)

This sub-directory contains scripts of commonly used path-planning algorithms which are: Dijkstra, A*, and RRT. It also contains an example of the Traveling Salesman Problem or TSP. 

![Dijkstra](https://user-images.githubusercontent.com/42896783/168454556-3b32b510-2fd9-4beb-9b18-2b7c6f697593.png)

![A_Star](https://user-images.githubusercontent.com/42896783/168454558-2482c041-0562-495a-b258-7660684d5446.png)

![RRT](https://user-images.githubusercontent.com/42896783/168454559-ac65647b-037e-4c94-b250-6dcd65c418be.png)

![Traveling Salesman Problem](https://user-images.githubusercontent.com/42896783/168454561-4f1814fb-ae18-49ed-8b2c-c47af63f3481.png)

## Evader and Pursuer
[Evader and Pursuer Sub-Directory](https://github.com/petern2599/Unmanned-Systems/tree/main/Evader%20and%20Pursuer)

This sub-directory contains the scripts which uses A* and proportional navigation with a simulated LIDAR sensor. This is facilitated in Gazebo and uses the middleware of ROS. It is used with two Turtlebots in Gazebo where one of them acts as the pursuer and the other as the evader. For the evader script it has an parameter option to enable evasive maneuvers where it essentially plans it route first, the create offset to each waypoint (assuming open area). This causes a zig-zag path which typically dodges the pursuer.

![evader and pursuer (no evasion)](https://user-images.githubusercontent.com/42896783/168454645-74b6528f-3e5f-46df-90b2-7bddb621fe0a.PNG)

![evader and pursuer](https://user-images.githubusercontent.com/42896783/168454647-df2d810a-2701-44d3-aba5-be09b3c3c9f8.PNG)

![evader_pursuer_gazebo](https://user-images.githubusercontent.com/42896783/168454655-daa31a8a-7703-489c-be00-4ab96ba051a3.PNG)
