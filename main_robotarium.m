%% Disturbance Estimation for GritsBot-Xs on the Robotarium Node (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.
%
% This experiment involves robots that estimate the disturbance online, and
% efficiently sample the environment by visiting the points in the state
% space with the highest variance estimate. 

% Initialization File
init;  

%% Initialize 
r                       = Robotarium('NumberOfRobots', N, 'ShowFigure', true); % Get Robotarium object used to communicate with the robots/simulator
cbf_wrapper             = CBFwrapper(N, n, m, CBF_MODE, SAFETY_RADIUS);
pose_controller         = create_minnorm_controller(); 
waypoint_node           = WaypointNode(N,n,m,CBF_MODE,COMM_MODE,IP,PORT);      % Disturbance Estimator
data_saver              = DataSaver(N,NOMINAL_RADIUS);                         % Data saving 
t_stamp                 = tic;

% Main Loop
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium (dt = 0.033)
    x               = r.get_poses(); 

    % Generate Waypoints (check if reached and updates)
    waypoint_node   = waypoint_node.waypoint_step(x);
    % Generate Robot inputs
    dxu             = pose_controller(x, waypoint_node.waypoints);
    dxu_nom         = dxu; % For saving purposes
    %% Collision Avoidance
    [mus_, sigmas_] = waypoint_node.predict(x');
    [dxu, min_h] = cbf_wrapper.uni_barrier_certificate(dxu, x, [], mus_, sigmas_);
    %% Append Data to be saved for GP and save trajectory data
    if mod(t,10) == 0
        waypoint_node = waypoint_node.append_traj_data(x, dxu, data_saver.x_old, data_saver.dxu_old);
    end
    if mod(t,50) == 0
        plot(x(1,:), x(2,:), 'bo', 'MarkerSize', 30, 'LineWidth', 5);
    end
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    
    %% Save old states to be used for data collection
    data_saver      = data_saver.save(x, dxu, toc(t_stamp), dxu_nom, min_h);
    % Save Data
    if mod(t,300) == 0
        save([SAVE_PATH, 'robotarium_data.mat'], 'waypoint_node', 'data_saver', 'cbf_wrapper');
    end
end

waypoint_node.plot_sigmas();
waypoint_node.clean_up();
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();
save([SAVE_PATH, 'robotarium_data.mat'], 'waypoint_node', 'data_saver', 'cbf_wrapper');
data_saver.plot_min_h(SAVE_PATH);
data_saver.plot_u_diff(SAVE_PATH);

