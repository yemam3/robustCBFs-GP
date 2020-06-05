classdef WaypointNode
    %DISTURBANCEESTIMATOR Estimates disturbance for control affine systems
    %   of the form: x_dot(t) = (g(x(t)) + D(x(t)))u(t)
    %   using Gaussian Processes.
    
    properties (Constant)
        % Transformation Matrix from Differential Drive to Unicycle
        pub_topic           = 'data';                       % Topic to publish to on MQTT
        sub_topic           = 'models';                     % Topic to subscribe to on MQTT
        bds                 = [-1.2, 1.2, -0.7, 0.7];       % Robotarium Bounds  
        dt                  = 0.033;                        % Robotarium Timestep
        granularity         = 0.25;                         % granularity of heatmap
    end
    properties
        comm_mode                                           % Options are 'MQTT', SharedFiles'
        cbf_mode                                            % CBF Mode (either 'Additive' or 'Multiplicative')
        mqtt_interface                                      % MQTT Interface
        N                                                   % Number of Robots
        n                                                   % Dimension of State
        m                                                   % Dimension of Input
        waypoints                                           % Current waypoints of the robots 
        waypoint_queue                                      % Queue of waypoints the robots should go to (always index first obj.n columns)
        data                                                % 3 x num_data_points (variable)
        uncertainty_grid                                    
        all_sigmas                                          % size(size(uncertainty_grid,1),n*m,-1) Record all sigmas every time we predict the meshgrid
        all_mus                                             % size(size(uncertainty_grid,1),n*m,-1)
        gpr_models                                          % cell(n x m)
        counter_models                                      % counter to update models
        deadlock_counter                                    % Deadlock Counter
    end
    
    methods
        function obj = WaypointNode(N, n, m, cbf_mode, comm_mode, ip, port)
            %WAYPOINTNODE Constructor of the class an instance of this class
            
            obj.comm_mode           = comm_mode;
            obj.cbf_mode            = cbf_mode;  
            % Setup MQTT Node
            if strcmp(comm_mode, 'MQTT')
                obj.mqtt_interface      = MqttInterface('waypoint_node', ip, port, 1);
                obj.mqtt_interface.subscribe(obj.sub_topic);
            else
                obj.mqtt_interface      = [];
            end

            obj.N                   = N;        
            obj.n                   = n;         
            obj.m                   = m;         
            obj.waypoints           = [];       
            obj.waypoint_queue      = [];
            % Intiailize Coordinates for uncertainty grid
<<<<<<< HEAD:classes/WaypointNode.m
            obj.uncertainty_grid    = build_uncertainty_grid(obj.bds,obj.granul_htmp); 
=======
            obj.uncertainty_grid    = build_uncertainty_grid(obj.bds, 'Granularity', obj.granularity); 
>>>>>>> origin/master:classes/WaypointNode.m
            if strcmp(cbf_mode, 'Additive')
                s = obj.n;
            elseif strcmp(cbf_mode, 'Multiplicative')
                s = obj.n * obj.m;
            elseif strcmp(cbf_mode, 'Regular')
                s = 0;
            end
            obj.all_mus             = zeros([size(obj.uncertainty_grid,1),s,0]);
            obj.all_sigmas          = zeros([size(obj.uncertainty_grid,1),s,0]);               
            % Data to be sent to the GP node to fit_rgp (resets once sent)
            obj.data                = zeros([0,2*obj.n+obj.m]); 
            % Gaussian Process Models
            obj.gpr_models          = [];
            obj.counter_models      = 0;
            obj.deadlock_counter    = zeros([1, obj.N]);
        end
        
        function obj = waypoint_step(obj, x)
            %WAYPOINT_STEP Checks which robot reached its waypoint. And
            %creates new waypoints for the robots which did.
            %   Inputs:
            %       x: 3xN matrix containing state of robots
            
            obj.counter_models       = obj.counter_models + 1;
            if mod(obj.counter_models, 100) == 0 && ~strcmp(obj.cbf_mode, 'Regular')
                obj                     = obj.update_models();
                obj.counter_models      = 0;
            end
            if isempty(obj.waypoints)
                ids                 = boolean(ones([1,obj.N]));
            else
                ids                = sum((x(1:2,:) - obj.waypoints(1:2,:)).^2, 1) < 0.15;
<<<<<<< HEAD:classes/WaypointNode.m
                %theta_diff          = (x(3,:) - obj.waypoints(3,:));
                %ids2                = abs(atan2(sin(theta_diff), cos(theta_diff))) < pi/4;
                %ids                 = (ids1 & ids2);
            end
            obj                     = obj.gen_next_waypoint(x, ids);
=======
                % Deadlock Mitigation
                ids                = ids | (obj.deadlock_counter > 10);
                obj.deadlock_counter(ids) = 0;
            end
            obj    = obj.gen_next_waypoint(x, ids);
>>>>>>> origin/master:classes/WaypointNode.m
        end
        
        function obj = gen_next_waypoint(obj, x, ids)
            %GEN_NEXT_WAYPOINT Generates next waypoints for each robot.
            %   Inputs:
            %       x: 3xN matrix containing state of robots
            %     ids: 1xN_free boolean vector containing mask of free robots
            %   Outputs:
            %       p: 2xN matrix containing next waypoints
            %
            %   We also want to make sure that the waypoints assigned are
            %   not close to each other.
            
            max_iter                = 50;       % Number of iterations we try to make sure new point is far away
            N_free                  = sum(ids);
            p                       = zeros([3,N_free]);
            ids                     = find(ids);
            
            for i = 1:N_free
                iter                = 1;
                while iter == 1 || ((any(sum((p(1:2,1:i-1) - p(1:2,i)).^2) < 0.35) || ~(isempty(obj.waypoints) || all(sum((obj.waypoints(1:2,~ids) - p(1:2,i)).^2) > 0.35))) && iter <= max_iter)
                    % If queue is empty assign random point
                    if isempty(obj.waypoint_queue)
                        p(:,i)                      = rand([obj.n,1]) - 0.5;
                        p(1,i)                      = (diff(obj.bds(1:2))) * p(1,i);   % Stretch x
                        p(2,i)                      = (diff(obj.bds(3:4))) * p(2,i);   % Stretch y
                        p(3,i)                      = 2*pi * p(3,i);                   % Stretch theta [-pi, pi]
                    % If not, assign next point from queue    
                    else
                        p(:,i)                      = obj.waypoint_queue(1, 1:obj.n)';
                        obj.waypoint_queue(1,:)     = [];
                    end
<<<<<<< HEAD:classes/WaypointNode.m
                    obj.waypoints(:,ids(i))              = p(:,i);
=======
                    obj.waypoints(:,ids(i))         = p(:,i);
>>>>>>> origin/master:classes/WaypointNode.m
                    iter = iter + 1;
                end
            end
        end
        
        function obj = deadlock_mitigation(obj, dxu)
            % DEADLOCK_MITIGATION
            % Increments deadlock counter for agents that did not move.    
                obj.deadlock_counter = obj.deadlock_counter + double(sum(abs(dxu), 1) < .05);
        end
        
        function obj = clear_traj_data(obj)
            obj.data  = zeros([0,2*obj.n+obj.m]);
        end
        
        function obj = append_traj_data(obj, x, dxu, x_old, dxu_old)
            %APPEND_TRAJ_DATA Sends data over MQTT to the GP node to be
            %used for disturbance estimation.
            % TODO: Add fake noise mode
            % Get New Data from MQTT under topic: obj.sub_topic
            ids                                     = 1:obj.N;                          % Going to always collect data instead 
            x_dot                                   = x(:,ids) - x_old(:,ids);
            x_dot(3,:)                              = atan2(sin(x_dot(3,:)), cos(x_dot(3,:)));
            x_dot                                   = x_dot / obj.dt;                   % Add Fake Noise Here
            u                                       = dxu_old(:,ids);
            new_data                                = [x_old(:,ids); x_dot; u]';        % new data shape: x x_dot u
            % Delete data with small u
            del_inds = (abs(u(1,:)) < 0.05) | (abs(u(2,:)) < 0.1);
<<<<<<< HEAD:classes/WaypointNode.m
            del_inds = del_inds | (abs(x_dot(3,:)) <= 0.1) | (abs(x_dot(3,:)) >= 1.5);
=======
            del_inds = del_inds | (abs(x_dot(3,:)) < 0.1) | (abs(x_dot(3,:)) > 1.5);
>>>>>>> origin/master:classes/WaypointNode.m
            new_data(del_inds,:)      = [];                                             % Prune data with 0 u
            obj.data(end+1:end+size(new_data,1),:)  = new_data;
            
            % Send then Clear Newly Collected Data Points 
            if size(obj.data,1) > 10 && ~strcmp(obj.cbf_mode, 'Regular')
                fprintf('Sending data...\n')
                if strcmp(obj.comm_mode, 'MQTT')
                % Send over MQTT
                    obj.mqtt_interface.send_json(obj.pub_topic, obj.data);
                elseif strcmp(obj.comm_mode, 'FileSharing')
                % Save in a File
                    temp = obj.data;
                    save('data.mat','temp');
                end
                obj = obj.clear_traj_data();       
            end
        end
        
        function obj = update_models(obj)
            %UPDATE_MODELS updates GP models.
            
            % Receive GP Models
            if strcmp(obj.comm_mode, 'MQTT')
            % Sent over MQTT
                [models, err, err_flag] = obj.mqtt_interface.receive_bytes(obj.sub_topic); % gp models cell shape [obj.n,obj.m]
            elseif strcmp(obj.comm_mode, 'FileSharing')
            % Saved in a File
                try 
                    models = load('models.mat');
                    models = models.temp;
<<<<<<< HEAD:classes/WaypointNode.m
                    %delete('models.mat');
=======
>>>>>>> origin/master:classes/WaypointNode.m
                catch e
                    e
                    models = [];
                end
            end

            % If no data is received then there is nothing to update
            if isempty(models) 
               return 
            end
            fprintf('Obtained Updated Models!\n')
            obj.gpr_models = models;
            % Update the estimates of the state space grid 
            obj = obj.update_heat_map();
        end
        
        function obj = update_heat_map(obj)
            %UPDATE_HEATMAP updates uncertainty grid and repopulates
            %waypoint queue.
            %
            % obj.waypoint_queue has the same size as the uncertainty grid,
            % except the row indices are sorted from highest to lowest
            % sigma (last column).
  
            [mus, sigmas]               = obj.predict(obj.uncertainty_grid(:,1:obj.n));
            obj.uncertainty_grid(:,4)   = sum(sigmas,2);
            obj.all_mus(:,:,end+1)      = mus;
            obj.all_sigmas(:,:,end+1)   = sigmas; 
            % Update Waypoint Queue
            [~, inds]                   = sort(-obj.uncertainty_grid(:,4));
            obj.waypoint_queue          = obj.uncertainty_grid(inds, :);
            % Remove Points the robots are currently going to
            for i = 1:obj.N
               obj.waypoint_queue(all(obj.waypoint_queue(:,1:3) == obj.waypoints(:,i)',2),:) = [];
            end
        end
        
        function [mus, sigmas] = predict(obj, x)
            %PREDICT uses the gps to predict the entries of D_1(x)
            % input: 
            %   - x: data to be trained of size(size(uncertainty_grid,1), obj.n)
            % output:
            %   - mus: means of predictions size(size(uncertainty_grid,1), obj.n * obj.m)
            %   - sigmas: stds of predictions size(size(uncertainty_grid,1), obj.n * obj.m)
            % Note that the entries are arranged column wise first.
            % e.g. for n = 3 and m = 2
            %   mus = [mus_11, mus_21, mus_31, mus_12, ...]
            %  where each mus_ij is a column vector of size -1 (num_data)
            
            assert(size(x,2) == obj.n, 'Columns of x must be of the size of the state x_i');
            
            
            % Initialize empty mus and sigmas
            mus    = zeros(size(x,1), numel(obj.gpr_models));
            sigmas = zeros(size(x,1), numel(obj.gpr_models));
            
            
            if strcmp(obj.cbf_mode, 'Regular') % Regular barriers assume no disturbance
                mus    = [];
                sigmas = [];
                return
            elseif isempty(obj.gpr_models) % If is empty just return prior 
                if strcmp(obj.cbf_mode, 'Multiplicative')
                    s      = obj.n * obj.m;
                    sig    = 0.005;
                elseif strcmp(obj.cbf_mode, 'Additive')
                    s      = obj.n;
                    sig    = 0.005;
                end
                mus    = zeros(size(x,1), s);
                sigmas = sig * ones(size(x,1), s);
                return
            end
            
            % Compute mus and sigmas for each one of the gps
            for i = 1:size(obj.gpr_models,1)
                for j = 1:size(obj.gpr_models,2)
                    if ~isempty(obj.gpr_models{i,j})
                        if iscell(obj.gpr_models{i,j}) % Deal with angles
                            [mus1_, sigmas1_]             = obj.gpr_models{i,j}{1}.predict(x); % sin of angle
                            [mus2_, sigmas2_]             = obj.gpr_models{i,j}{2}.predict(x); % cos of angle
                            mus_                          = atan2(mus1_, mus2_);
                            sigmas_                       = max(sigmas1_, sigmas2_); 
                        else
                            [mus_, sigmas_]             = obj.gpr_models{i,j}.predict(x);
                        end
                            mus(:,(j-1)*obj.n+i)        = mus_;
                            sigmas(:,(j-1)*obj.n+i)     = sigmas_;
                    end
                end
            end
        end  
                    
        function plot_sigmas(obj, save_path)
            %PLOT_SIGMAS plots sigmas (variance) of the disturbance
            %estimation over time. 
            
            if isempty(obj.gpr_models)
                return
            end
            
            % Add Colors so that they're constant through different plots
            colors = ['b','g','r','c','m','y','k','w']; % TODO: Add Colors 
            if length(size(obj.all_sigmas)) ~= 3
                return
            end
            fig = figure(1000);
            hold on; grid on;
            ax          = gca;
            ax.FontSize = 20;
            ylabel('$\max\sigma_{i,j}$','Interpreter','latex','FontSize', 30);
            xlabel('iteration','Interpreter','latex','FontSize', 30);
            max_sigmas  = squeeze(max(obj.all_sigmas, [], 1));
            lgd_entries = cell([1,size(obj.all_sigmas,2)]);
            % Plot sigmas vs iterations for each of the gp models (n*m) 
            for j = 1:size(obj.gpr_models,2)
                for i = 1:size(obj.gpr_models,1)
                    plot(max_sigmas((j-1)*obj.n+i,:), colors((j-1)*obj.n+i), 'LineWidth', 5);
                    lgd_entries{(j-1)*obj.n+i} = ['$\sigma_{',num2str(i),num2str(j),'}$'];
                end
            end
            legend(lgd_entries, 'Interpreter','latex','FontSize',20,'Location','northeast');
            hold off;
            savefig(fig, [save_path, 'sigma_plot.fig']);
        end
        
        function animate_spatiotemp_mean_var(obj, save_path)
            
            generate_grid_animation(obj.bds, obj.granularity, squeeze(sum(obj.all_sigmas,2)), 'SavePath', [save_path, 'sigma_animation.gif'], 'ZTitle', '$\sum_{i,j} \sigma_{i,j}$');
            generate_grid_animation(obj.bds, obj.granularity, squeeze(sum(obj.all_mus,2)), 'SavePath', [save_path, 'mu_animation.gif'], 'ZTitle', '$\sum_{i,j} \mu_{i,j}$');
        end
        
        function obj = clean_up(obj)
            %CLEAN_UP Delete any temporary files used for experiment 
            
            if strcmp(obj.comm_mode, 'FileSharing') && ~strcmp(obj.cbf_mode, 'Regular')
                delete('data.mat')
                delete('models.mat')
            end
            
        end
<<<<<<< HEAD:classes/WaypointNode.m
        
        function obj = clean_up(obj)
            %CLEAN_UP Delete any temporary files used for experiment 
            
            if strcmp(obj.comm_mode, 'FileSharing') && ~strcmp(obj.cbf_mode, 'Regular')
                delete('data.mat')
                delete('models.mat')
            end
            
        end
=======
>>>>>>> origin/master:classes/WaypointNode.m
    end
end

