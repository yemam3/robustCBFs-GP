%% Disturbance Estimation of GritsBot-X on the Robotarium (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.
init; init_mqtt; date_string = datestr(datetime('now'),'HH:MM:SS.FFF'); rng(); 
%% Get Robotarium object used to communicate with the robots/simulator
N = 3;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
% Intialize Controllers and Safety Functions
uni_barrier_certificate = create_uni_barrier_certificate_with_boundary();
si_position_controller = create_si_position_controller_v2();

% Disturbance Estimator
waypoint_node = WaypointNode(N,3,2);
x_old = []; x_data = zeros(3,N,0); u_data = zeros(2,N,0);

% Select the number of iterations for the experiment.  
iterations = 10000;

% Main Loop
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium (dt = 0.033)
    x               = r.get_poses();
    
    %% Compute Waypoints
    % Generate Waypoints (check if reached and updates)
    waypoint_node   = waypoint_node.waypoint_step(x);
    % Generate Robot inputs
    dxu             = si_position_controller(waypoint_node.waypoints, x);
    %disp(disturb_estimator.waypoints)
    % Collision Avoidance
    dxu             = uni_barrier_certificate(dxu, x, []); 

    % Save data
    if mod(t,40) == 0
        waypoint_node = waypoint_node.append_traj_data(x, dxu, x_old, dxu_old);
        plot(x(1,:), x(2,:), 'bo', 'MarkerSize', 30, 'LineWidth', 5);
        save(['saved_data/main_mqtt_workspace_', date_string,'.mat'], 'waypoint_node', 'x_data', 'u_data');
    end
    
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    % Save old states to be used for data collection
    x_old           = x;
    dxu_old         = dxu;
    x_data          = cat(3,x_data, x);
    u_data          = cat(3,u_data, dxu);
end
waypoint_node.plot_max_sigmas();
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();
close all;

