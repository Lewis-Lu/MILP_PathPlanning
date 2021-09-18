# This is the ampl file for the 2D MILP path planning
# Multi agents for collision avoidance
# This is the part of the code for the partial requirement for the degree of M.Sc. Robotics
# 
# Author: Lu, Hong
# E-mail: hlu39@sheffield.ac.uk
# If you want to use the code for own purpose, please contact the author for agreement.
# Last Modified: July 24, 2021

param n_agent integer >=1; # number of the agents

param n_ts integer >2; # number of time steps

param dt > 0; # non-integer for the dt

param n_obs integer >=0; # number of the obstacles

# obstacle information centroid and radius 
param obs_centroid{1..n_obs, 1..2}; # obstacle centroid
param obs_radius{1..n_obs}; # obstacle radius
param obs_inflation >= 1; # coefficient for obstacle inflation

param n_vel_cst integer >=2; # number of the velocity constraints
param n_acc_cst integer >=2; # number of the acceleration constraints
param n_obs_cst integer >=2; # number of the obstacle constraints
param n_agt_cst integer >=2; # number of the agent constraints

param acc_max >0; # maximum acceleration
param vel_max >0; # maximum velocity
param vel_min >0, <= vel_max; # minimum velocity

param pos_init{1..2, 1..n_agent}; # initial position for each agent
param vel_init{1..2, 1..n_agent}; # initial velocity for each agent
param pos_final{1..4, 1..n_agent}; # final position circle for each agent [xmin xmax ymin ymax]

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

# approximation for the agent
param cos_agt{1..n_agt_cst};
param sin_agt{1..n_agt_cst};

# for the under-estimation usage
param cos_vmax;
param cos_amax;

# variables
var pos{1..2, 1..n_agent, 1..n_ts}; # position
var vel{1..2, 1..n_agent, 1..n_ts}; # velocity
var acc{1..2, 1..n_agent, 1..(n_ts-1)}; # acceleration

var finish{1..n_ts, 1..n_agent} binary; # 1 for finish, 0 otherwise. Both for each agent i-th agent finished at time step t

var obs_avoid{1..n_ts, 1..n_agent, 1..n_obs, 1..n_obs_cst} binary; # decision variable for the obstacle avoidance logic. Both for each agent

# decision variable for the collision avoidance logic.
# collision avoidance in four directions
var col_avoid{1..n_ts, 1..n_agent, 1..n_agent, 1..n_agt_cst};  

minimize time_consumption: sum{i in 1..n_agent, t in 1..n_ts} t*finish[t,i];

# initial state
subject to initpos{i in 1..2, j in 1..n_agent}: pos[i,j,1] = pos_init[i,j]; # initial position for each agent
subject to initvel{i in 1..2, j in 1..n_agent}: vel[i,j,1] = vel_init[i,j]; # initial velocity for each agent

# kinematics and dynamics constraints
subject to kinematics{i in 1..2, j in 1..n_agent, k in 1..(n_ts-1)}: vel[i,j,k+1] = vel[i,j,k] + acc[i,j,k]*dt;
subject to dynamics{i in 1..2, j in 1..n_agent, k in 1..(n_ts-1)}: pos[i,j,k+1] = pos[i,j,k] + vel[i,j,k]*dt + 0.5*acc[i,j,k]*dt*dt;

# position box constraints
subject to position_cst_min{i in 1..2, j in 1..n_agent, k in 1..n_ts}: pos[i,j,k] >= pos_cst[2*i-1];
subject to position_cst_max{i in 1..2, j in 1..n_agent, k in 1..n_ts}: pos[i,j,k] <= pos_cst[2*i];

# subject to velocity_cst_min{i in 1..(n_ts-1)}: vel[1,i]*vel[1,i] + vel[2,i]*vel[2,i] >= (vel_min^2)/(vel_max^2);
subject to velocity_cst{i in 1..(n_ts-1), j in 1..n_vel_cst, k in 1..n_agent}: vel[1,k,i]*sin_vel[j] + vel[2,k,i]*cos_vel[j] <= vel_max*cos_vmax;
subject to accelerate_cst{i in 1..(n_ts-1), j in 1..n_acc_cst, k in 1..n_agent}: acc[1,k,i]*sin_acc[j] + acc[2,k,i]*cos_acc[j] <= acc_max*cos_amax;

# arrival check (arrival box check)
subject to arrival_cst_hi{t in 1..n_ts, i in 1..2, j in 1..n_agent}: pos[i,j,t] >= pos_final[2*i-1,j] - (pos_final[2*i-1,j] - pos_cst[2*i-1])*(1-finish[t,j]);
subject to arrival_cst_lo{t in 1..n_ts, i in 1..2, j in 1..n_agent}: pos[i,j,t] <= pos_final[2*i,j] - (pos_final[2*i,j] - pos_cst[2*i])*(1-finish[t,j]);
subject to arrival_logic{i in 1..n_agent}: sum{t in 1..n_ts} finish[t,i] = 1;

# obstacle avoidance
# subject to obstacle_cst{i in 1..n_ts, j in 1..n_obs, k in 1..n_obs_cst, p in 1..n_agent}: (pos[1,i]-obs_centroid[j,1])*sin_obs[k] + (pos[2,i] - obs_centroid[j,2])*cos_obs[k] >= obs_radius[j] - 1000*obs_avoid[i,j,k];
# subject to obstacle_avoidance_logic{i in 1..n_ts, j in 1..n_obs, p in 1..n_agent}: sum{k in 1..n_obs_cst} obs_avoid[i,j,k] <= n_obs_cst-1;

# collision avoidance
subject to collision_avoidance{i in 1..n_ts, p in 1..n_agent, q in 1..n_agent, k in 1..n_agt_cst}: if  q > p then (pos[1,q,i] - pos[1,p,i])*sin_agt[k] + (pos[2,q,i] - pos[2,p,i])*cos_agt[k] >= 0.5 - 1000*col_avoid[i,p,q,k];
subject to collision_avoidance_logic{i in 1..n_ts, p in 1..n_agent, q in 1..n_agent}: sum{k in 1..n_agt_cst} col_avoid[i,p,q,k] <= n_agt_cst-1;
