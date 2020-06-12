%% Disturbance Estimation for GritsBot-Xs on the Robotarium Node (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.
% 
% This experiment involves robots aligned in a circle, that repeatedly
% switch positions.

% Initialization File
init; mkdir(SAVE_PATH); 

%% Initialize 
show_figure             = true;
r                       = Robotarium('NumberOfRobots', N, 'ShowFigure', show_figure); % Get Robotarium object used to communicate with the robots/simulator
cbf_wrapper             = CBFwrapper(N, n, m, CBF_SPECS);
position_control        = create_si_position_controller();
si_to_uni_dyn           = create_si_to_uni_dynamics();
waypoint_node           = WaypointNode(N,n,m,CBF_SPECS.cbf_mode,COMM_MODE,IP,PORT);      % Disturbance Estimator
data_saver              = DataSaver(N,CBF_SPECS.nominal_radius);                         % Data saving
exp_logger              = setup_logger(2, exp_date, CBF_SPECS);
t_stamp                 = tic;

%% Plotting Trajectory Traces
if show_figure
    trace_handle = cell([N,1]);
    for i = 1:N
        trace_handle{i} = plot([0],[0], 'b-', 'LineWidth', 15);
    end
end

%% Experiment constants 
% Next, we set up some experiment constants
% This code ensures that the agents are initially distributed around an
% ellipse.  
xybound = [-1, 1, -0.8, 0.8];
p_theta = (1:2:2*N)/(2*N)*2*pi;
p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];
x_goal = p_circ(:,1:N);
flag = 0; %flag of task completion

if ~strcmp(CBF_SPECS.cbf_mode, 'Regular') 
    waypoint_node = waypoint_node.update_models();
end

%% Main Loop
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium (dt = 0.033)
    x               = r.get_poses(); 
    if IS_SIM
        Os(1,:) = cos(x(3, :)); 
        Os(2,:) = sin(x(3, :));
        x(1:2, :) = x(1:2, :) - CBF_SPECS.projection_distance * Os;
    end
    % Generate Robot inputs
    x_temp = x(1:2,:);
    %% Algorithm
    % Let's make sure we're close enough the the goals
    if norm(x_goal-x(1:2,:),1)<0.03
         flag = 1-flag;
    end
    
    % This code makes the robots switch positions on the ellipse
    if flag == 0
        x_goal = p_circ(:,1:N);
    else
        x_goal = p_circ(:,N+1:2*N);
    end    
        
    % Use a single-integrator position controller to drive the agents to
    % the circular formation
    dx = position_control(x(1:2, :), x_goal);

    %% Apply barrier certs. and map to unicycle dynamics   
    % Transform the single-integrator dynamics to unicycle dynamics using a
    % diffeomorphism, which can be found in the utilities
    dxu_nom = si_to_uni_dyn(dx, x);   
    %% Collision Avoidance
    [mus_, sigmas_] = waypoint_node.predict(x');
    [dxu, min_h, dt_cbf] = cbf_wrapper.uni_barrier_certificate(dxu_nom, x, [], mus_, sigmas_);
    %% Append Data to be saved for GP and save trajectory data
    %if mod(t,10) == 0
     %   waypoint_node = waypoint_node.append_traj_data(x, dxu, data_saver.x_old, data_saver.dxu_old);
     %   waypoint_node = waypoint_node.waypoint_step(x);
    %end
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    % Send the previously set velocities to the agents.  This function must be called!
    r.step_no_error();
    %% Save old states to be used for data collection
    data_saver      = data_saver.save(x, dxu, toc(t_stamp), dxu_nom, min_h, dt_cbf);
    %% Update Trace Plots
    if show_figure
        for i = 1:N
            trace_handle{i}.XData = data_saver.x_data(1,i,max(1,t-200):end);
            trace_handle{i}.YData = data_saver.x_data(2,i,max(1,t-200):end);
        end
    end
    % Save Data
    if mod(t,300) == 0
        save([SAVE_PATH, 'robotarium2_data.mat'], 'waypoint_node', 'data_saver', 'cbf_wrapper');
    end
end

%% Generate Plots and save data
waypoint_node.plot_sigmas(SAVE_PATH);
waypoint_node.animate_spatiotemp_mean_var(SAVE_PATH);
%waypoint_node.clean_up();
save([SAVE_PATH, 'robotarium2_data.mat'], 'waypoint_node', 'data_saver', 'cbf_wrapper');
data_saver.plot_min_h(SAVE_PATH);
u_diff = data_saver.plot_u_diff(SAVE_PATH);
exp_logger.logMesg(['Mean CBF solution Time = ' num2str(mean(data_saver.dt_cbf_data))], 1);
exp_logger.logMesg(['Var CBF solution Time  = ' num2str(var(data_saver.dt_cbf_data))], 1);
exp_logger.logMesg(['Max CBF solution Time  = ' num2str(max(data_saver.dt_cbf_data))], 1);
exp_logger.logMesg(['mean(||u^* - u_{nom}||^2) = ' num2str(mean(u_diff))], 1);
exp_logger.logMesg(['var(||u^* - u_{nom}||^2)  = ' num2str(var(u_diff))], 1);
exp_logger.logMesg(['max(||u^* - u_{nom}||^2)  = ' num2str(max(u_diff))], 1);
exp_logger.save_messages([SAVE_PATH, 'logger.txt']);
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();