%% Disturbance Estimation for GritsBot-Xs on the Robotarium Node (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.
%
% This experiment involves robots that estimate the disturbance online,  and
% efficiently sample the environment by visiting the points in the state
% space with the highest variance estimate. The top left quarter of the
% environment has a multiplicative distubance that multiplies the input by
% 1.2.

% Initialization File
<<<<<<< HEAD
init;  
=======
init;  mkdir(SAVE_PATH); rng(50);
>>>>>>> origin/master

%% Initialize
r                       = Robotarium('NumberOfRobots', N, 'ShowFigure', true); % Get Robotarium object used to communicate with the robots/simulator
h=fill([0,0,-1.6,-1.6], [0,1.0,1.0,0], [1,1,0]);
h.FaceAlpha=0.3;
cbf_wrapper             = CBFwrapper(N, n, m, CBF_SPECS);
pose_controller         = create_minnorm_controller(); 
waypoint_node           = WaypointNode(N,n,m,CBF_SPECS.cbf_mode,COMM_MODE,IP,PORT);      % Disturbance Estimator
data_saver              = DataSaver(N,CBF_SPECS.nominal_radius);                         % Data saving 
t_stamp                 = tic;

% Main Loop
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium (dt = 0.033)
    x               = r.get_poses(); 
    if IS_SIM
        Os(1,:) = cos(x(3, :)); 
        Os(2,:) = sin(x(3, :));
        x(1:2, :) = x(1:2, :) - CBF_SPECS.projection_distance * Os;
    end
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
    %% Save old states to be used for data collection
    data_saver      = data_saver.save(x, dxu, toc(t_stamp), dxu_nom, min_h);
    % Save Data
    if mod(t,300) == 0
        save([SAVE_PATH, 'robotarium_data.mat'], 'waypoint_node', 'data_saver', 'cbf_wrapper');
    end
    %% Add Multiplicative Disturbance
<<<<<<< HEAD
    dxu(:, x(1,:) < 0 & x(2,:) > 0) = 1.20 * dxu(:, x(1,:) < 0 & x(2,:) > 0);
=======
    dxu(:, x(1,:) < 0 & x(2,:) > 0) = 0.80 * dxu(:, x(1,:) < 0 & x(2,:) > 0);
>>>>>>> origin/master
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
<<<<<<< HEAD
end

waypoint_node.plot_sigmas();
=======
    waypoint_node = waypoint_node.deadlock_mitigation(dxu);
end

waypoint_node.plot_sigmas(SAVE_PATH);
waypoint_node.animate_spatiotemp_mean_var(SAVE_PATH);
>>>>>>> origin/master
waypoint_node.clean_up();
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();
save([SAVE_PATH, 'robotarium_data.mat'], 'waypoint_node', 'data_saver', 'cbf_wrapper');
data_saver.plot_min_h(SAVE_PATH);
data_saver.plot_u_diff(SAVE_PATH);

