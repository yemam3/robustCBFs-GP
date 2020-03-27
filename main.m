%% Disturbance Estimation of GritsBot-X on the Robotarium (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.
rng()
close all; clear all; clc;
%% Get Robotarium object used to communicate with the robots/simulator
N = 1;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
% Intialize Controllers and Safety Functions
uni_barrier_certificate = create_uni_barrier_certificate_with_boundary();
si_position_controller = create_si_position_controller_v2();

% Disturbance Estimator
disturb_estimator = DisturbanceEstimator(N,3,2);
x_old = [];

% Select the number of iterations for the experiment.  
iterations = 10000;

% Main Loop
for t = 1:iterations
    
    
    disturb_estimator = disturb_estimator.fit_data();

     
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Compute Waypoints
    % Generate Waypoints (check if reached and updates)
    disturb_estimator = disturb_estimator.waypoint_step(x);
    % Generate Robot inputs
    dxu = si_position_controller(disturb_estimator.waypoints, x);
    %disp(disturb_estimator.waypoints)
    % Collision Avoidance
    dxu = uni_barrier_certificate(dxu, x, []); 
    % Save data
    if t > 1
        disturb_estimator = disturb_estimator.append_traj_data(x, dxu, x_old, dxu_old);
    end
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    % Save old states to be
    x_old = x;
    dxu_old = dxu;
end
disturb_estimator.plot_max_sigmas();
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();