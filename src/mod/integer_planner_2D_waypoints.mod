# This is the ampl file for the 2D MILP path planning
# No support for the multi-robot assignment and collision avoidance
# Support for the single vehicle for multi-waypoint targets with obstacle avoidance
# This is the part of the code for the partial requirement for the degree of M.Sc. Robotics

# Author: Lu, Hong
# E-mail: hlu39@sheffield.ac.uk
# Last Modified: July 23, 2021

# If you want to use the code for own purpose, please contact the author for agreement.

param epsilon >0; # weighting on control input
param n_ts integer >2; # number of time steps
param n_obs integer >=0; # number of the obstacles
param n_wp integer >=0; # number of the waypoints

# waypoint information
param waypoints{1..4, 1..n_wp};

# obstacle information centroid and radius 
param obs_centroid{1..n_obs, 1..2};
param obs_radius{1..n_obs};

param obs_inflation >= 1; # coefficient for obstacle inflation

param n_vel_cst integer >=2; # number of the velocity constraints
param n_acc_cst integer >=2; # number of the acceleration constraints
param n_obs_cst integer >=2; # number of the obstacle constraints

param dt > 0; # non-integer for the dt
param acc_max >0; # maximum acceleration
param vel_max >0; # maximum velocity
param vel_min >0, <= vel_max; # minimum velocity

param pos_init{1..2};
param vel_init{1..2};

param pos_final{1..4}; # final position circle [xmin xmax ymin ymax]

param pos_cst{1..4}; # position constraints [xmin xmax ymin ymax]

# approximation for the velocity
param cos_vel{1..n_vel_cst};
param sin_vel{1..n_vel_cst};

# approximation for the acceleration
param cos_acc{1..n_acc_cst};
param sin_acc{1..n_acc_cst};

# approximation for the obstacle 
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

# var finish{1..n_ts} binary; # 1 for finish, 0 otherwise

# var obs_avoid{1..n_ts, 1..n_obs, 1..n_obs_cst} binary; # decision variable for the obstacle avoidance logic

var binary_wp{1..n_ts, 1..n_wp} binary; #decision variable for waypoint visit;

# minimize time_consumption: sum{t in 1..n_ts} t*finish[t];

minimize time_consumption: sum{t in 1..n_ts, p in 1..n_wp} t*binary_wp[t,p] + epsilon*sum{t in 1..(n_ts-1)} am[t];

# initial state
subject to initpos{i in 1..2}: pos[i,1] = pos_init[i];
subject to initvel{i in 1..2}: vel[i,1] = vel_init[i];

# kinematics and dynamics constraints
subject to kinematics{i in 1..2, j in 1..(n_ts-1)}: vel[i,j+1] = vel[i,j] + acc[i,j]*dt;
subject to dynamics{i in 1..2, j in 1..(n_ts-1)}: pos[i,j+1] = pos[i,j] + vel[i,j]*dt + 0.5*acc[i,j]*dt*dt;

# position box constraints
subject to position_cst_min{i in 1..2, j in 1..n_ts}: pos[i,j] >= pos_cst[2*i-1];
subject to position_cst_max{i in 1..2, j in 1..n_ts}: pos[i,j] <= pos_cst[2*i];

# subject to velocity_cst_min{i in 1..(n_ts-1)}: vel[1,i]*vel[1,i] + vel[2,i]*vel[2,i] >= (vel_min^2)/(vel_max^2);

subject to velocity_cst{i in 1..(n_ts-1), j in 1..n_vel_cst}: vel[1,i]*sin_vel[j] + vel[2,i]*cos_vel[j] <= vel_max*cos_vmax;
subject to accelerate_cst{i in 1..(n_ts-1), j in 1..n_acc_cst}: acc[1,i]*sin_acc[j] + acc[2,i]*cos_acc[j] <= acc_max*cos_amax;
subject to accelerate_cst_am{i in 1..(n_ts-1), j in 1..n_acc_cst}: acc[1,i]*sin_acc[j] + acc[2,i]*cos_acc[j] <= am[i];

# arrival check (arrival box check)
# subject to arrival_cst_hi{t in 1..n_ts, i in 1..2}: pos[i,t] >= pos_final[2*i-1] - (pos_final[2*i-1] - pos_cst[2*i-1])*(1-finish[t]);
# subject to arrival_cst_lo{t in 1..n_ts, i in 1..2}: pos[i,t] <= pos_final[2*i] - (pos_final[2*i] - pos_cst[2*i])*(1-finish[t]);
# subject to arrival_logic: sum{t in 1..n_ts} finish[t] = 1;

# obstacle avoidance
# subject to obstacle_cst{i in 1..n_ts, j in 1..n_obs, k in 1..n_obs_cst}: (pos[1,i]-obs_centroid[j,1])*sin_obs[k] + (pos[2,i] - obs_centroid[j,2])*cos_obs[k] >= obs_radius[j] - 1000*obs_avoid[i,j,k];
# subject to obstacle_avoidance_logic{i in 1..n_ts, j in 1..n_obs}: sum{k in 1..n_obs_cst} obs_avoid[i,j,k] <= n_obs_cst-1;

# waypoint check
subject to waypoint_visit_lo{i in 1..n_ts, j in 1..n_wp, k in 1..2}: pos[k,i] >= waypoints[2*k-1, j] - (waypoints[2*k-1, j] - pos_cst[2*k-1])*(1-binary_wp[i,j]);
subject to waypoint_visit_hi{i in 1..n_ts, j in 1..n_wp, k in 1..2}: pos[k,i] <= waypoints[2*k, j] - (waypoints[2*k, j] - pos_cst[2*k])*(1-binary_wp[i,j]);
subject to waypoint_visit_logic{i in 1..n_wp}: sum{j in 1..n_ts} binary_wp[j, i] = 1;
