%% Disturbance Estimation for GritsBot-Xs on the Robotarium Node (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.

% Initialization File
init;  

%% Get Robotarium object used to communicate with the robots/simulator
r                       = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
% Intialize Controllers and Safety Functions
if strcmp(CBF_MODE, 'Multiplicative')
    uni_barrier_certificate = create_uni_barrier_certificate_with_boundary_mult('SafetyRadius', SAFETY_RADIUS);
elseif strcmp(CBF_MODE, 'Additive')
    uni_barrier_certificate = create_uni_barrier_certificate_with_boundary_add('SafetyRadius', SAFETY_RADIUS);
elseif strcmp(CBF_MODE, 'Regular')
    uni_barrier_certificate = create_uni_barrier_certificate_with_boundary_reg('Disturbance',0,'SafetyRadius', SAFETY_RADIUS);
else 
    error('CBF_MODE needs to be either Multiplicative or Additive! Check init file.')
end
pose_controller         = create_minnorm_controller(); %create_minnorm_waypoint_controller();
% Disturbance Estimator
waypoint_node           = WaypointNode(N,n,m,CBF_MODE,COMM_MODE,IP,PORT);
x_old                   = []; 
x_data                  = zeros(3,N,0); 
u_data                  = zeros(2,N,0);
t_data                  = [];
min_h_data              = [];
t_stamp                 = tic;

% Main Loop
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium (dt = 0.033)
    x               = r.get_poses(); 
    if IS_SIM 
        x               = x + 0.0005*(randn(size(x)));
    end
    
    %% Compute Waypoints
    % Generate Waypoints (check if reached and updates)
    waypoint_node   = waypoint_node.waypoint_step(x);
    % Generate Robot inputs
    dxu             = pose_controller(x, waypoint_node.waypoints);
    % Collision Avoidance
    if strcmp(CBF_MODE, 'Multiplicative')
        if ~isempty(waypoint_node.gpr_models)
            [mus_, sigmas_] = waypoint_node.predict(x');
            mus_            = reshape(mus_',[n,m,N]);
            sigmas_         = reshape(sigmas_',[n,m,N]);
            [dxu, min_h]    = uni_barrier_certificate(dxu, x, [], mus_ - 2*sigmas_, mus_ + 2*sigmas_); 
        else
            [dxu, min_h]    = uni_barrier_certificate(dxu, x, [], -0.1*ones([n,m,N]), 0.1*ones([n,m,N]));
        end
    elseif strcmp(CBF_MODE, 'Additive')
        if ~isempty(waypoint_node.gpr_models)
            [mus_, sigmas_] = waypoint_node.predict(x');
            mus_            = reshape(mus_',[n,N]);
            sigmas_         = reshape(sigmas_',[n,N]);
            [dxu, min_h]    = uni_barrier_certificate(dxu, x, [], mus_ - 2*sigmas_, mus_ + 2*sigmas_); 
        else
            [dxu, min_h]    = uni_barrier_certificate(dxu, x, [], -0.05*ones([n,N]), 0.05*ones([n,N]));
        end
    else % No Disturbance
        [dxu, min_h]        = uni_barrier_certificate(dxu, x, []);
    end
    %% Append Data to be saved for GP and save trajectory data
    if mod(t,10) == 0
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
    min_h_data      = cat(1,min_h_data,min_h - NOMINAL_RADIUS^2);
    % Save Data
    if mod(t,300) == 0
        save(['saved_data/robotarium_', CBF_MODE, '_', num2str(SAFETY_RADIUS*100), '_', date_string,'.mat'], 'waypoint_node', 'x_data', 'u_data', 'min_h_data');
    end
end

waypoint_node.plot_sigmas();
waypoint_node.clean_up();
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();
save(['saved_data/robotarium_', CBF_MODE, '_', num2str(SAFETY_RADIUS*100), '_', date_string,'.mat'], 'waypoint_node', 'x_data', 'u_data', 'min_h_data');
figure(1001);hold on;grid on;
xlabel('iter'); ylabel('min_{ij}(h_ij(t))');
plot(1:iterations,min_h_data,'b'); 
plot(1:iterations,zeros(1,iterations),'r')