param Nt integer >2; #number of time steps
param No integer >=0; #number of obstacles
param Nc integer >=0; #number of obstacle constraints
param Nv integer >2; #number of constraints per speed circle
param Na integer >2; #number of constraints per acc circle

param dt >0; #time step length

param amax >0; #max force
param vmax >0; #max speed
param vmin >=0, <vmax; #min speed

param ri{1..2}; #initial position
param vi{1..2}; #initial velocity

param rf{1..4}; #final position box [xmin xmax ymin ymax]

param ro{1..4}; #max position box [xmin xmax ymin ymax]

param obst{1..Nc,1..4}; #obstacle data, row=[i,pxi,pyi,qi]

param M{1..Nc}; #big M for relaxation

param gamma >0; #weighting on force

param costa{1..Na}; #cosines for acc circles
param sinta{1..Na}; #sines for acc circles

param costv{1..Nv}; #cosines for speed circles
param sintv{1..Nv}; #sines for speed circles

var r{1..2,1..Nt}; #positions
var v{1..2,1..Nt}; #velocities
var a{1..2,1..(Nt-1)}; #accelerations
var f{1..Nt} binary; #1 for finish now, 0 otherwise
var b{1..Nc,1..(Nt-1)} binary; #binaries for avoidance
var am{1..(Nt-1)}; #force magnitudes

# inefficient cost function.
# minimize arrival: sum{t in 1..Nt} t*f[t];

# input penalty applied for solution
minimize arrival: sum{t in 1..Nt} t*f[t] + gamma*sum{t in 1..(Nt-1)} am[t];

# initial conditions

subject to initpos{i in 1..2}: r[i,1] = ri[i];

subject to initvel{i in 1..2}: v[i,1] = vi[i];

# system

subject to dynamics{t in 1..(Nt-1), i in 1..2}: vmax*v[i,t+1] = vmax*v[i,t] + amax*a[i,t]*dt;

subject to kinematics{t in 1..(Nt-1), i in 1..2}: r[i,t+1] = r[i,t] + vmax*v[i,t]*dt + 0.5*amax*a[i,t]*dt*dt;

# acc limits

subject to maxacc{t in 1..(Nt-1), i in 1..Na}: a[1,t]*costa[i] + a[2,t]*sinta[i] <= 1;

# force magnitude

subject to accmag{t in 1..(Nt-1), i in 1..Na}: a[1,t]*costa[i] + a[2,t]*sinta[i] <= am[t];

# speed limits

subject to maxspd{t in 2..(Nt-1), i in 1..Nv}: v[1,t]*costv[i] + v[2,t]*sintv[i] <= 1;

# min speed

var spdb{t in 2..(Nt-1), i in 1..Nv} binary;

subject to minspd{t in 2..(Nt-1), i in 1..Nv}: v[1,t]*costv[i] + v[2,t]*sintv[i] >= vmin/vmax - (vmin/vmax + 1)*spdb[t,i];

subject to minspdlog{t in 2..(Nt-1)}: sum{i in 1..Nv} spdb[t,i] <= Nv-1;

# position limits

subject to minpos{t in 2..Nt, i in 1..2}: r[i,t] >= ro[2*i-1];

subject to maxpos{t in 2..Nt, i in 1..2}: r[i,t] <= ro[2*i];

# arrival checks

subject to arrival_lo{t in 1..Nt, i in 1..2}: r[i,t] >= rf[2*i-1] - (rf[2*i-1] - ro[2*i-1])*(1-f[t]);

subject to arrival_hi{t in 1..Nt, i in 1..2}: r[i,t] <= rf[2*i] + (ro[2*i] - rf[2*i])*(1-f[t]);

subject to arrival_logic: sum{t in 1..Nt} f[t] = 1;

# avoidance

subject to avoid{k in 1..Nc, i in 1..Nc, t in 1..(Nt-1)}: obst[k,2]*r[1,t] + obst[k,3]*r[2,t] >= obst[k,4] - 100*b[k,t];

subject to avoid_logic{k in 1..No, t in 1..(Nt-1)}: sum{i in 1..Nc: obst[i,1]==k} b[i,t] <= (sum{i in 1..Nc: obst[i,1]==k} 1) - 1 + (sum{s in 1..(t-1)} f[s]);