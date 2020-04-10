%% Disturbance Estimation of GritsBot-X on the Robotarium (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.


% Init Files that add requirements to path
%init;  
init_mqtt;
% For Data Saving Purposes
date_string = datestr(datetime('now'),'HH:MM:SS.FFF'); rng(); 
% Suppress is not serializable warning (caused when saving data)
warning('off', 'MATLAB:Java:ConvertFromOpaque');
%% Get Robotarium object used to communicate with the robots/simulator
N                       = 3; % Number of Robots
n                       = 3; % Dimension of state x_i 
m                       = 2; % Dimension of control u_i
r                       = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
% Intialize Controllers and Safety Functions
uni_barrier_certificate = create_uni_barrier_certificate_with_boundary_v2();
si_position_controller  = create_minnorm_waypoint_controller();
% Select the number of iterations for the experiment.  
iterations              = 10000;
% Disturbance Estimator
waypoint_node           = WaypointNode(N,n,m);
x_old                   = []; 
x_data                  = zeros(3,N,0); 
u_data                  = zeros(2,N,0);
t_data                  = [];
t_stamp                 = tic;

% Main Loop
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium (dt = 0.033)
    x               = r.get_poses();
    
    %% Compute Waypoints
    % Generate Waypoints (check if reached and updates)
    waypoint_node   = waypoint_node.waypoint_step(x);
    % Generate Robot inputs
    dxu             = si_position_controller(waypoint_node.waypoints, x);
    % Collision Avoidance
    if ~isempty(waypoint_node.gpr_models)
        [mus_, sigmas_] = waypoint_node.predict(x');
        mus_            = reshape(mus_',[n,m,N]);
        sigmas_         = reshape(sigmas_',[n,m,N]);
        dxu             = uni_barrier_certificate(dxu, x, [], mus_ - 2*sigmas_, mus_ + 2*sigmas_);
    else
        dxu             = uni_barrier_certificate(dxu, x, [], -0.01*ones(n,m,N), 0.01*ones(n,m,N));
    end
    %% Append Data to be saved for GP and save trajectory data
    if mod(t,100) == 0
        waypoint_node = waypoint_node.append_traj_data(x, dxu, x_old, dxu_old);
        plot(x(1,:), x(2,:), 'bo', 'MarkerSize', 30, 'LineWidth', 5);
    end
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    
    %% Save old states to be used for data collection
    x_old           = x;
    dxu_old         = dxu;
    x_data          = cat(3,x_data, x);
    u_data          = cat(3,u_data, dxu);
    t_data          = toc(t_stamp);
    % Save Data
    if mod(t,300) == 0
        save(['saved_data/main_mqtt_workspace_', date_string,'.mat'], 'waypoint_node', 'x_data', 'u_data', 't_data');
    end
end

waypoint_node.plot_sigmas();
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();
save(['saved_data/main_mqtt_workspace_', date_string,'.mat'], 'waypoint_node', 'x_data', 'u_data');