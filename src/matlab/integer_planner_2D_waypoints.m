function ofile = integer_planner_2D_waypoints(pos_init, vel_init, waypoints, obs_info, opbox, num_vel_cst,...
    num_acc_cst, num_obs_cst, acc_max, vel_max, vel_min, num_ts, dt, out_data_file)    
% This function is integer planner in 2D environment, and the function
% returns the results from the solver.
%
% ofile = integer_planner_2D(pos_init,vel_init, pos_final, num_vel_cst, opbox,...
%       num_acc_cst, acc_max, vel_max, vel_min, num_ts, dt, out_data_file)

% Version 1.0 : Lu, Hong, 17 July 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 22 July 2021
    
    disp('Mixed Integer Linear Planner for path planning.')
    % assertion for the input
    assert(length(pos_init) == 2, 'Dimension of the initial position should be 2')
    assert(length(vel_init) == 2, 'Dimension of the initial velocity should be 2')
    
    % assertion for the waypoints
    if size(waypoints, 1) == 0
        error('The Waypoints should be given')
    else 
        assert(size(waypoints, 2) == 4, 'Dimension of the waypoint should be 4 [Xwi_min, Xwi_max, Ywi_min, Ywi_max] for i-th waypoint')
    end
    
    assert(length(num_vel_cst) == 1, 'Number of the velocity approximation constraint should be a scalar')
    assert(length(num_acc_cst) == 1, 'Number of the acceleration approximation constraint should be a scalar')
    assert(length(num_obs_cst) == 1, 'Number of the obstacle approximation constraint should be a scalar')
    assert(length(acc_max) == 1, 'Maximum acceleration should be a scalar')
    assert(length(vel_max) == 1, 'Maximum velocity should be a scalar')
    assert(length(vel_min) == 1, 'Minimum velocity should be a scalar')
    assert(floor(num_ts) == num_ts, 'Number of the time step should be scalar and integer')
    assert(length(dt) == 1, 'The time gap value should be a scalar')
    assert(length(opbox) == 4, 'The length of the opbox should be 4')
    
    if ~isempty(obs_info)
        assert(size(obs_info, 2) == 3, 'Column dimension of the obstacle should be 3, [x, y, radius]')
        num_obs = size(obs_info, 1);
    else
        num_obs = 0;
        disp('No obstacle is given')
    end
    
    file_root = 'D:\Postgraduate\Dissertation\src\'; % change the file root according to the custom PC
    
    filename = [[file_root 'data\'] out_data_file '.dat'];
    fid = fopen(filename, 'w');
    ctn = 0;
    ctn = ctn + AMPLcomment(fid, ['Data file generated for ' out_data_file]);
    
    % write epsilon
    ctn = ctn + AMPLscalar(fid, 'epsilon', 0.01);
    
    % write the circle approximation
    ctn = ctn + AMPLscalarint(fid, 'n_vel_cst', num_vel_cst);
    ctn = ctn + AMPLscalarint(fid, 'n_acc_cst', num_acc_cst);
    ctn = ctn + AMPLvector(fid, 'cos_vel', cos(2*pi*[1:num_vel_cst]/num_vel_cst));
    ctn = ctn + AMPLvector(fid, 'sin_vel', sin(2*pi*[1:num_vel_cst]/num_vel_cst));
    ctn = ctn + AMPLvector(fid, 'cos_acc', cos(2*pi*[1:num_acc_cst]/num_acc_cst));
    ctn = ctn + AMPLvector(fid, 'sin_acc', sin(2*pi*[1:num_acc_cst]/num_acc_cst));
    ctn = ctn + AMPLscalar(fid, 'cos_vmax', cos(pi/num_vel_cst));
    ctn = ctn + AMPLscalar(fid, 'cos_amax', cos(pi/num_acc_cst));
    
    % write waypoint information
    ctn = ctn + AMPLscalar(fid, 'n_wp', size(waypoints, 1));
    ctn = ctn + AMPLmatrix(fid, 'waypoints', waypoints');
    
    % write obstacle information
    if num_obs ~= 0
        ctn = ctn + AMPLmatrix(fid, 'obs_centroid', obs_info(:,1:2));
        ctn = ctn + AMPLvector(fid, 'obs_radius', obs_info(:,3));
        ctn = ctn + AMPLscalarint(fid, 'n_obs', size(obs_info, 1));
        ctn = ctn + AMPLscalarint(fid, 'n_obs_cst', num_obs_cst);
        ctn = ctn + AMPLvector(fid, 'cos_obs', cos(2*pi*[1:num_obs_cst]/num_obs_cst));
        ctn = ctn + AMPLvector(fid, 'sin_obs', sin(2*pi*[1:num_obs_cst]/num_obs_cst));
    end
    
    % write the vel and acc limiations
    ctn = ctn + AMPLscalar(fid, 'acc_max', acc_max);
    ctn = ctn + AMPLscalar(fid, 'vel_max', vel_max);
    ctn = ctn + AMPLscalar(fid, 'vel_min', vel_min);
    
    % write time information
    ctn = ctn + AMPLscalarint(fid, 'n_ts', num_ts);
    ctn = ctn + AMPLscalar(fid, 'dt', dt);
    
    % write pos_init and pos_final
    ctn = ctn + AMPLvector(fid, 'pos_init', pos_init);
    ctn = ctn + AMPLvector(fid, 'vel_init', vel_init);
%     ctn = ctn + AMPLvector(fid, 'pos_final', pos_final);
    ctn = ctn + AMPLvector(fid, 'pos_cst', opbox);
    
    fclose(fid);
    
    sprintf('%d bytes written', ctn)
    disp(['Data file generation finished. File located at ' filename])
    
    % call the solver for solutions
    model_root = [file_root 'mod\'];
    solver_root = [file_root 'glpk-5.0\w64\'];
    data_root = [file_root 'data\'];
    output_root = [file_root 'output\'];
    
    solver = [solver_root 'glpsol.exe '];
    model = [model_root 'integer_planner_2D_waypoints.mod '];
    data = [data_root 'integer_planner_2D_waypoints.dat '];
    ofile = [output_root 'integer_planner_2D_waypoints.txt '];
    
    cmd = [solver '--model ' model '--data ' data '--output ' ofile];
    
    statue = system(cmd);
    
    if statue == 0
        disp('solver is called.')
    else
        disp('solver is not called.')
    end

end