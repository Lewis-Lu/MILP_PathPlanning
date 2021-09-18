# Path Planning Using Mixed Integer Linear Programming



This project focuses on the theme of path planning using MILP. I took this as my M.Sc. Dissertation project under supervision of Dr. Paul Trodden.



## Abstract

A comprehensive study on the Mixed Integer Linear Programming (MILP) path planning is carried out under the mathematical programming framework. Various control architectures for different purposes are studied, implemented and compared in this paper. The concept of the MILP is introduced at the beginning, then its combination with the path planning is illustrated in detail along with its applications under separated scenarios considering various requirements, such as collision avoidance between the agents, obstacle avoidance and waypoint handling when multiple tasks existed. Moreover, iterative algorithm is also studied from two different perspectives in this paper which significantly reduce the computation time. Apart from global planning, the control architecture with the combination of the local planning scheme is also studied under the framework of Receding Horizon Control. The different resolutions of the MILP-RHC enables the vehicle not only to navigate through the environment safely and without falling into the entrapment using the high-level map information, but also to merely consider the low-level sensed environment. Such online fashioned planning relieves the computational burden significantly. In the last section, the extended capabilities of the MILP path planner is also studied both in finer trajectory generation for Quadrotor UAV and in 3D climber as well. Extensive simulations and results are shown in each section for the MILP path planning studies.



## Structure

### 1. One-time global planning using MILP 

Ref [1] has given the detailed guidance, the whole implementation in this project is illustrated in the *dissertation* pdf, the code is available in *matlab* folder for various scenarios. Current MILP solver is GLPK and CPLEX. The C/C++ version using COPT （杉树科技） solver would be proposed later.



#### Example: Single Vehicle with Obstacle Avoidance

<img src="src\img\image-20210918172039225.png" alt="image-20210918172039225" style="zoom: 80%;" />

<img src="src\img\image-20210918192621042.png" alt="image-20210918192621042" style="zoom:80%;" />

#### Example: Collision Avoidance

<img src="src\img\image-20210918192711499.png" alt="image-20210918192711499" style="zoom: 80%;" />

#### Example: Multiple Waypoint Handling

<img src="src\img\image-20210918192800146.png" alt="image-20210918192800146" style="zoom: 67%;" />

<img src="src\img\image-20210918192815439.png" alt="image-20210918192815439" style="zoom: 50%;" />

#### Example: Wind Disturbance (with perfect knowledge of the disturbance)

<img src="src\img\image-20210918192927631.png" alt="image-20210918192927631" style="zoom: 67%;" />

<img src="src\img\image-20210918211228143.png" alt="image-20210918211228143" style="zoom: 50%;" />

### 2. Iterative Time Selection / Iterative Obstacle Growing Algorithm



Aiming to reduce the computation time of one-time global planning for the path, the iterative algorithm is proposed for relieving such burden. The collision check point quantity is shrinking down significantly compared to the fixed step size simulation.



#### Iterative Time Selection Algorithm

<img src="src\img\image-20210918211345439.png" alt="image-20210918211345439" style="zoom:80%;" />



#### Iterative Obstacle Growing Algorithm

<img src="src\img\image-20210918211405038.png" alt="image-20210918211405038" style="zoom:80%;" />

### 3. MPC-MILP (Receding Horizon Control - Mixed Integer Linear Planning)



#### MPC framework 

1. The MILP is used to optimize, starting from current state **s_cur** and current time **t** to stop at the time **t + H** at target state **s_tar**. **H** is the prediction horizon.

2. Only implement the first step of the control sequence resulting from the optimizer.

3. Repeat until reach the goal.

   <img src="src\img\MPC_1.gif" alt="MPC_1" style="zoom:67%;" />

<img src="src\img\MPC_2.gif" alt="MPC_2" style="zoom:67%;" />

#### Cost Function Design in MILP to avoid local minima

<img src="src\img\image-20210918213056586.png" alt="image-20210918213056586" style="zoom:67%;" />

### Extended Capacity

1. **Forest Flight for crazyflie quadrotor**

<img src="src\img\quadrotor.gif" alt="quadrotor" style="zoom:67%;" />

<img src="src\img\image-20210918213320797.png" alt="image-20210918213320797" style="zoom:67%;" />

2. **3D wall climber**

   <img src="src\img\image-20210918213409489.png" alt="image-20210918213409489" style="zoom:67%;" />

   <img src="src\img\image-20210918213420624.png" alt="image-20210918213420624" style="zoom:67%;" />

## Reference

[1] A. Richards and J.P. How. **Aircraft trajectory planning with collision avoidance using mixed integer linear programming**. In *Proceedings of the 2002 American Control Conference (IEEE Cat. No.CH37301)*, volume 3, pages 1936–1941 vol.3, 2002.

