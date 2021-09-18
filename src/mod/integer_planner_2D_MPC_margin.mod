# This is the ampl file for the 2D MILP path planning with coorperation with MPC
# 
# Time Horizon Control
# 
# This is the part of the code for the partial requirement for the degree of M.Sc. Robotics
#
# Author: Lu, Hong
# E-mail: hlu39@sheffield.ac.uk
# If you want to use the code for own purpose, please contact the author for agreement.
# Last Modified: Aug 27, 2021

param epsilon >0; # weighting on control input

param dt > 0; # non-integer for the dt
param n_ts integer >2; # number of time steps
param n_ts_plan integer >0; # planning horizon

param pos_init{1..2};
param vel_init{1..2};

# operation box (opbox)
param pos_cst{1..4}; # position constraints [xmin xmax ymin ymax]
# goal box
param pos_final{1..4}; # final position circle [xmin xmax ymin ymax]

# wind disturbance
param wind_disturbance{1..2}; # wind disturbance at each time step in m/s

param pi := 3.14159265;
param M := 1000;

# obstacle information
param n_obs integer >=0;
param obs{1..n_obs, 1..4}; # [xmin, ymin, xmax, ymax]
# #coefficient for obstacle inflation
# param obs_inflation >= 1; 

param Rm{1..2};
param Vm{1..2};
param Fm{1..2};

param n_vel_cst integer >=2; # number of the velocity constraints
param n_acc_cst integer >=2; # number of the acceleration constraints
param n_obs_cst integer >=2; # number of the obstacle constraints

param acc_max >0; # maximum acceleration
param vel_max >0; # maximum velocity
param vel_min >0, <=vel_max; # minimum velocity

# approximation for the velocity
param cos_vel{1..n_vel_cst};
param sin_vel{1..n_vel_cst};

# approximation for the acceleration
param cos_acc{1..n_acc_cst};
param sin_acc{1..n_acc_cst};

# approximation for the obstacle 
# reserved field, rectangle obstacle is default as 4
param cos_obs{1..n_obs_cst};
param sin_obs{1..n_obs_cst};

# for the under-estimation usage
param cos_vmax;
param cos_amax;

# variables
var pos{1..2, 1..n_ts};
var vel{1..2, 1..n_ts};
var acc{1..2, 1..(n_ts-1)};

var am{1..(n_ts-1)}; #force magnitudes
var finish{1..n_ts} binary; # 1 for finish, 0 otherwise
var obs_avoid{1..n_ts, 1..n_obs, 1..n_obs_cst} binary; # decision variable for the obstacle avoidance logic
var b{1..n_obs, 1..n_obs_cst, 1..n_ts} binary; 

# variable is not valid for abs() operate
# minimize distanceToGoal: abs(pos_final[1]-pos[1,n_ts]) + abs(pos_final[2] - pos[2,n_ts])

minimize arr: sum{t in 1..n_ts} t*finish[t] + epsilon*sum{t in 1..(n_ts-1)} am[t];

# initial state
subject to initpos{i in 1..2}: pos[i,1] = pos_init[i];

subject to initvel{i in 1..2}: vel[i,1] = vel_init[i];

# kinematics and dynamics constraints

subject to kinematics_1{i in 1..2}: vel[i,2] = vel[i,1] + (acc[i,1] + wind_disturbance[i]) * dt;
subject to kinematics{i in 1..2, j in 2..(n_ts-1)}: vel[i,j+1] = vel[i,j] + acc[i,j]*dt;

subject to dynamics_1{i in 1..2}: pos[i,2] = pos[i,2] + vel[i,1]*dt + 0.5*(acc[i,1] + wind_disturbance[i])*dt*dt;
subject to dynamics{i in 1..2, j in 2..(n_ts-1)}: pos[i,j+1] = pos[i,j] + vel[i,j]*dt + 0.5*acc[i,j]*dt*dt;

# position box constraints

subject to position_cst_min{i in 1..2, j in 1..n_ts}: pos[i,j] >= pos_cst[2*i-1];
subject to position_cst_max{i in 1..2, j in 1..n_ts}: pos[i,j] <= pos_cst[2*i];

subject to accelerate_cst_am{i in 1..(n_ts-1), j in 1..n_acc_cst}: acc[1,i]*sin_acc[j] + acc[2,i]*cos_acc[j] <= am[i];

# subject to accelerate_cst{i in 1..(n_ts-1), j in 1..n_acc_cst}: acc[1,i]*sin_acc[j] + acc[2,i]*cos_acc[j] <= acc_max*cos_amax;

# acceleration limits with margins

subject to maxforce1{i in 1..n_acc_cst}: acc[1,1]*cos_acc[i] + acc[2,1]*sin_acc[i] <= acc_max;

subject to maxforce2{i in 1..n_acc_cst}: acc[1,2]*cos_acc[i] + acc[2,2]*sin_acc[i] <= acc_max - Fm[1];

subject to maxforce3{t in 3..(n_ts-1), i in 1..n_acc_cst}: acc[1,t]*cos_acc[i] + acc[2,t]*sin_acc[i] <= acc_max - Fm[2];

# subject to velocity_cst_min{i in 1..(n_ts-1), j in 1..n_vel_cst}: vel[1,i]*sin_vel[j] + vel[2,i]*cos_vel[j] >= vel_min*cos_vmax;

# subject to velocity_cst{i in 1..(n_ts-1), j in 1..n_vel_cst}: vel[1,i]*sin_vel[j] + vel[2,i]*cos_vel[j] <= vel_max*cos_vmax;

# speed limits with margins

subject to maxspeed1{i in 1..n_vel_cst}: vel[1,1]*cos_vel[i] + vel[2,1]*sin_vel[i] <= vel_max*cos_vmax;

subject to maxspeed2{i in 1..n_vel_cst}: vel[1,2]*cos_vel[i] + vel[2,2]*sin_vel[i] <= vel_max*cos_vmax - Vm[1];

subject to maxspeed3{t in 3..n_ts, i in 1..n_vel_cst}: vel[1,t]*cos_vel[i] + vel[2,t]*sin_vel[i] <= vel_max*cos_vmax - Vm[2];


# arrival check (arrival box check)

# subject to arrival_cst_lo{t in 1..n_ts, i in 1..2}: pos[i,t] >= pos_final[2*i-1] - 1000*(1-finish[t]);

# subject to arrival_cst_hi{t in 1..n_ts, i in 1..2}: pos[i,t] <= pos_final[2*i] + 1000*(1-finish[t]);

# subject to arrival_logic: sum{t in 1..n_ts} finish[t] = 1;

# arrival check (arrival box check) with margins

subject to arrival_lo1{i in 1..2}: pos[i,1] >= pos_final[2*i-1] - M*(1-finish[1]);

subject to arrival_hi1{i in 1..2}: pos[i,1] <= pos_final[2*i] + M*(1-finish[1]);

subject to arrival_lo2{i in 1..2}: pos[i,2] >= pos_final[2*i-1] + Rm[1] - M*(1-finish[2]);

subject to arrival_hi2{i in 1..2}: pos[i,2] <= pos_final[2*i] - Rm[1] + M*(1-finish[2]);

subject to arrival_lo3{t in 3..n_ts, i in 1..2}: pos[i,t] >= pos_final[2*i-1] + Rm[2] - M*(1-finish[t]);

subject to arrival_hi3{t in 3..n_ts, i in 1..2}: pos[i,t] <= pos_final[2*i] - Rm[2] + M*(1-finish[t]);

subject to arrival_logic: sum{t in 1..n_ts} finish[t] = 1;

# obstacle avoidance
# If not considered the obstacles, the following code can be commented

# subject to obstacle_cst_x_lo{t in 1..n_ts, j in 1..n_obs}: pos[1,t] <= obs[j,1] + 1000*obs_avoid[t,j,1];

# subject to obstacle_cst_x_hi{t in 1..n_ts, j in 1..n_obs}: pos[1,t] >= obs[j,3] - 1000*obs_avoid[t,j,2];

# subject to obstacle_cst_y_lo{t in 1..n_ts, j in 1..n_obs}: pos[2,t] <= obs[j,2] + 1000*obs_avoid[t,j,3];

# subject to obstacle_cst_y_hi{t in 1..n_ts, j in 1..n_obs}: pos[2,t] >= obs[j,4] - 1000*obs_avoid[t,j,4];

# # at least one direction should satisfied
# subject to obstacle_avoidance_logic{i in 1..n_ts, j in 1..n_obs}: sum{k in 1..n_obs_cst} obs_avoid[i,j,k] <= n_obs_cst-1; 

# avoidance with margin

subject to avoid_lo1{k in 1..n_obs, i in 1..2}: pos[i,1] <= obs[k,i] + M*b[k,i,1];

subject to avoid_hi1{k in 1..n_obs, i in 1..2}: pos[i,1] >= obs[k,i+2] - M*b[k,i+2,1];

subject to avoid_lo2{k in 1..n_obs, i in 1..2}: pos[i,2] <= obs[k,i] - Rm[1] + M*b[k,i,2];

subject to avoid_hi2{k in 1..n_obs, i in 1..2}: pos[i,2] >= obs[k,i+2] + Rm[1] - M*b[k,i+2,2];

subject to avoid_lo3{k in 1..n_obs, t in 3..n_ts, i in 1..2}: pos[i,t] <= obs[k,i] - Rm[2] + M*b[k,i,t];

subject to avoid_hi3{k in 1..n_obs, t in 3..n_ts, i in 1..2}: pos[i,t] >= obs[k,i+2] + Rm[2] - M*b[k,i+2,t];

subject to avoid_logic{k in 1..n_obs, t in 1..n_ts}: sum{i in 1..4} b[k,i,t] <= 3 + (sum{s in 1..(t-1)} finish[s]); # if finished, no obstacle avoidance needed.
