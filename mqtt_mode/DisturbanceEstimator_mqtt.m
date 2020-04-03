classdef DisturbanceEstimator_mqtt
    %DISTURBANCEESTIMATOR Estimates disturbance for control affine systems
    %   of the form: x_dot(t) = (g(x(t)) + D(x(t)))u(t)
    %   using Gaussian Processes.
    
    properties (Constant)
        pub_topic                   = 'sigmas';
        sub_topic                   = 'data';
        bds                         = [-1.4, 1.4, -0.8, 0.8, -pi, pi];  % state space boundaries
        granul_htmp                 = 0.25;                             % granularity of heatmap
        threshold_data_num          = 10;                               % min # of new data required to recompute gp models
    end
    properties
        mqtt_interface
        N
        n
        m
        data                                                            % 3 x num_data_points (variable)
        gpr_models                                                      % cell(n x m)
        uncertainty_grid                                                % meshgrid size(-1,obj.n+1)        
        num_new_data                                                    % number of new data points acquired
        % For Data Saving Purposes
        all_sigmas
        all_mus
    end
    
    methods
        function obj = DisturbanceEstimator_mqtt(N, n, m)
            %DisturbanceEstimator Construct an instance of this class
            
            % Setup MQTT Node
            % Robotarium: mqttInterface = MqttInterface('matlab_node', '192.168.1.8', 1884); 
            % Localhost:  mqttInterface = MqttInterface('matlab_node', 'localhost', 1883); 
            obj.mqtt_interface      = MqttInterface('gp_node', 'localhost', 1883, 1);
            obj.mqtt_interface.subscribe(obj.sub_topic);
            obj.N                   = N;  
            obj.n                   = n;
            obj.m                   = m;
            obj.data                = zeros([0,2*obj.n+obj.m]);
            obj.num_new_data        = 0;
            % Intiailize Uncertainty Grid
            obj.uncertainty_grid    = build_uncertainty_grid(obj.bds,obj.granul_htmp);
            obj.all_mus             = zeros([size(obj.uncertainty_grid,1),obj.n*obj.m,0]);
            obj.all_sigmas          = zeros([size(obj.uncertainty_grid,1),obj.n*obj.m,0]);
        end
        
        function obj = main(obj)
            %MAIN runs one iteration 
            obj = obj.append_traj_data();
            obj = obj.fit_data();
        end
        
        function obj = fit_data(obj)
            %fit_data fits gpr models for g(x)
            %
            % x_dot = [x_dot; y_dot; theta_dot] = g(x) [v; omega]
            % where g(x) = [sin(theta), 0
            %               cos(theta), 0
            %               0,          1]
            % We solely fit a gpr for non-zero entries as of now.
            % Output:
            %   obj.grp_models cell(N,M)
            %   
            
            if isempty(obj.data) || obj.num_new_data < obj.threshold_data_num
                return
            end
            obj.gpr_models                  = cell(obj.n, obj.m);
            x                               = obj.data(:,1:obj.n);
            x_dot                           = obj.data(:,obj.n+1:2*obj.n);
            u                               = obj.data(:,2*obj.n+1:end);
            % g(x)_11
            y_11                            = x_dot(:,1) ./ u(:,1) - cos(x(:,3));
            obj.gpr_models{1,1}             = fitrgp(x, y_11); 
            % g(x)_21
            y_21                            = x_dot(:,2) ./ u(:,1) - sin(x(:,3));
            obj.gpr_models{2,1}             = fitrgp(x, y_21); 
            % g(x)_32
            y_32                            = x_dot(:,3) ./ u(:,2) - 1;
            obj.gpr_models{3,2}             = fitrgp(x, y_32); 
            % Recompute Uncertainty Grid Based on  
            obj = update_heat_map(obj);
            obj.num_new_data = 0;
        end
        
        function obj = clear_traj_data(obj)
            obj.data  = zeros([0,2*obj.n+obj.m]);
        end
        
        function obj = append_traj_data(obj)
            %APPEND_TRAJ_DATA appends trajectory data obtained through mqtt
            
            % Get New Data from MQTT under topic: obj.sub_topic
            new_data = obj.mqtt_interface.receive_json(obj.sub_topic); % new data shape (-1,2n+m): [x_i, x_dot_i, u_i]
            if ~isempty(new_data)
                fprintf('Received new data!\n')
            end
            obj.data(end+1:end+size(new_data,1),:)  = new_data;        % Append new data to stored data
            obj.num_new_data = obj.num_new_data + size(new_data,1);    % count how much new data we've gathered so far
        end
        
        function obj = update_heat_map(obj)
            %UPDATE_HEAT_MAP generates sigmas for uncertainty grid
            %
            % Uses the predict function to get the uncertainty for the each
            % point in the uncertainty grid (state space mesh).
  
            [mus, sigmas]               = predict(obj, obj.uncertainty_grid(:,1:obj.n));
            obj.uncertainty_grid(:,4)   = sum(sigmas,2);
            obj.all_mus(:,:,end+1)      = mus;
            obj.all_sigmas(:,:,end+1)   = sigmas; 
            
            % Upon Updating the Sigmas for the heatmap publish those Sigmas
            obj.mqtt_interface.send_json(obj.pub_topic, obj.uncertainty_grid(:,4));
        end
        
        function [mus, sigmas] = predict(obj, x)
            %PREDICT uses the gps to predict the entries of D_1(x)
            % input: 
            %   - x: data to be trained of size(-1, obj.n)
            % output:
            %   - mus: means of predictions size(-1, obj.n * obj.m)
            %   - sigmas: stds of predictions size(-1, obj.n * obj.m)
            % Note that the entries are arranged column wise first.
            % e.g. for n = 3 and m = 2
            %   mus = [mus_11, mus_21, mus_31, mus_12, ...]
            %  where each mus_ij is a column vector of size -1 (num_data)
            
            assert(size(x,2) == obj.n, 'Columns of x must be of the size of the state x_i');
            % Initialize empty mus and sigmas
            mus    = zeros(size(x,1), obj.n * obj.m);
            sigmas = zeros(size(x,1), obj.n * obj.m);
            % Compute mus and sigmas for each one of the gps
            for i = 1:obj.n
                for j = 1:obj.m
                    if ~isempty(obj.gpr_models{i,j})
                        [mus_, sigmas_]             = obj.gpr_models{i,j}.predict(obj.uncertainty_grid(:,1:3));
                        mus(:,(j-1)*obj.n+i)        = mus_;
                        sigmas(:,(j-1)*obj.n+i)     = sigmas_;
                    end
                end
            end
        end  
                    
        function plot_sigmas(obj)
            % Add Colors so that they're constant through different plots
            colors = ['b','g','r','c','m','y','k','w']; % TODO: Add Colors 
            if length(size(obj.all_sigmas)) ~= 3
                return
            end
            size(obj.all_sigmas)
            figure(1000)
            hold on; grid on;
            ax = gca;
            ax.FontSize = 20;
            ylabel('$\sum\limits_{i,j}^{}\sigma_{i,j}$','Interpreter','latex','FontSize', 30);
            xlabel('iteration','Interpreter','latex','FontSize', 30);
            max_sigmas = squeeze(max(obj.all_sigmas, [], 1));
            size(max_sigmas)
            lgd_entries = cell([1,obj.n*obj.m]);
            for i = 1:obj.n
                for j = 1:obj.m
                    disp((j-1)*obj.n+i)
                    plot(max_sigmas((j-1)*obj.n+i,:), colors((j-1)*obj.n+i), 'LineWidth', 5);
                    lgd_entries{(j-1)*obj.n+i} = ['$\sigma_{',num2str(i),num2str(j),'}$'];
                end
            end
            legend(lgd_entries, 'Interpreter','latex','FontSize',20,'Location','northeast');
            hold off;
        end
            
        
    end
end

