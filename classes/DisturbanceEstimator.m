classdef DisturbanceEstimator
    %DISTURBANCEESTIMATOR Estimates disturbance for control affine systems
    %   of the form: x_dot(t) = (g(x(t)) + D(x(t)))u(t)
    %   using Gaussian Processes.
    
    properties (Constant)
        pub_topic                   = 'models';                         % topic to publish to on mqtt
        sub_topic                   = 'data';                           % topic to subscribe to on mqtt
        granul_htmp                 = 0.25;                             % granularity of heatmap
        threshold_data_num          = 10;                               % min # of new data required to recompute gp models
    end
    properties
        comm_mode                                                       % Options are 'MQTT', SharedFiles'
        cbf_mode                                                        % CBF Mode (either 'Additive' or 'Multiplicative')
        mqtt_interface                                                  % MQTT Interface
        N                                                               % Number of Robots
        n                                                               % Dimension of State
        m                                                               % Dimension of Input
        data                                                            % 3 x num_data_points (variable)
        gpr_models                                                      % cell(n x m)
        uncertainty_grid                                                % meshgrid size(size(uncertainty_grid,1),obj.n+1)        
        num_new_data                                                    % number of new data points acquired
        % For Data Saving Purposes
        is_sim                                                          % Sim (1) or Real (0)?
        fake_noise                                                      % Add fake noise if is_sim
    end
    
    methods
        function obj = DisturbanceEstimator(N, n, m, is_sim, cbf_mode, comm_mode, ip, port)
            %DisturbanceEstimator Construct an instance of this class
            
            obj.comm_mode           = comm_mode;
            obj.cbf_mode            = cbf_mode;
            % Robotarium: mqttInterface = MqttInterface('matlab_node', ); 
            % Localhost:  mqttInterface = MqttInterface('matlab_node', 'localhost', 1883); 
            % Setup MQTT Node
            if strcmp(comm_mode, 'MQTT')
                obj.mqtt_interface      = MqttInterface('gp_node', ip, port, 1);
                obj.mqtt_interface.subscribe(obj.sub_topic);
            else
                obj.mqtt_interface      = [];
            end
                
            obj.N                   = N;  
            obj.n                   = n;
            obj.m                   = m;
            obj.data                = zeros([0,2*obj.n+obj.m]);
            obj.num_new_data        = 0;
            obj.is_sim              = is_sim;
            if is_sim
                obj.fake_noise      = zeros([0,n*m]);
            else
                obj.fake_noise      = [];
            end
        end
        
        function obj = main(obj)
            %MAIN runs one iteration 
            obj = obj.append_traj_data();
            obj = obj.fit_data();
        end
        
        function obj = fit_data(obj)
            %fit_data fits gpr models for g(x)
            %
            % 
            % x_dot = [x_dot; y_dot; theta_dot] = g(x) [v; omega]
            % cbf_mode == 'Multiplicative':
            % where g(x) = [sin(theta), 0     [gpr_11,      0
            %               cos(theta), 0   +  gpr_21,      0
            %               0,          1]          0, gpr_32]
            % We solely fit a gpr for non-zero entries as of now.
            % cbf_mode == 'Additive':
            % where f(x) = [gpr_1
            %               gpr_2
            %               gpr_3]
            % Output:
            %   obj.grp_models cell(N,M)
            %   
            
            if isempty(obj.data) || obj.num_new_data < obj.threshold_data_num
                return
            end
            fprintf('Fitting New Models!!!!!\n');
            x                               = obj.data(:,1:obj.n);
            x_dot                           = obj.data(:,obj.n+1:2*obj.n);
            u                               = obj.data(:,2*obj.n+1:end);
            y                               = zeros(size(x_dot,1), obj.n*obj.m);
            if strcmp(obj.cbf_mode, 'Multiplicative')
                obj.gpr_models                  = cell(obj.n, obj.m);
                % g(x)_11
                y(:,1)                          = x_dot(:,1) ./ u(:,1) - cos(x(:,3)); 
                % g(x)_21
                y(:,2)                          = x_dot(:,2) ./ u(:,1) - sin(x(:,3));
                % g(x)_32
                y(:,6)                          = x_dot(:,3) ./ u(:,2) - 1;
                % Add fake noise if this is a simulation
                y = y + obj.fake_noise; % will be all 0s if is_sim =s= 0
                % Fit Model
                obj.gpr_models{1,1}             = fitrgp(x, y(:,1)); 
                obj.gpr_models{2,1}             = fitrgp(x, y(:,2)); 
                obj.gpr_models{3,2}             = fitrgp(x, y(:,6)); 
            elseif strcmp(obj.cbf_mode, 'Additive')
                obj.gpr_models                  = cell(obj.n, 1);
                % f(x)_1
                y(:,1)                          = x_dot(:,1) - cos(x(:,3)) .* u(:,1); 
                % f(x)_2
                y(:,2)                          = x_dot(:,2) - sin(x(:,3)) .* u(:,1);
                % f(x)_3
                y(:,6)                          = x_dot(:,3) - u(:,2);
                % Add fake noise if this is a simulation
                y = y + obj.fake_noise; % will be all 0s if is_sim =s= 0
                % Fit Model
                obj.gpr_models{1}               = fitrgp(x, y(:,1)); 
                obj.gpr_models{2}               = fitrgp(x, y(:,2)); 
                obj.gpr_models{3}               = fitrgp(x, y(:,6)); 
            else
                error('cbf_mode needs to be either Additive or Multiplicative! Check init file.')
            end
            
            % Reset Count of New Data 
            obj.num_new_data = 0;
            % Send the Updated Model
            if strcmp(obj.comm_mode, 'MQTT')
            % Upong Updating the GP Models Sent Them over MQTT
                obj.mqtt_interface.send_bytes(obj.pub_topic, obj.gpr_models);
            elseif strcmp(obj.comm_mode, 'FileSharing')
                temp = obj.gpr_models;
                save('models.mat', 'temp');
            end
        end
        
        function obj = clear_traj_data(obj)
            obj.data  = zeros([0,2*obj.n+obj.m]);
        end
        
        function obj = append_traj_data(obj)
            %APPEND_TRAJ_DATA appends trajectory data obtained through mqtt
            
            % Load The New Data
            if strcmp(obj.comm_mode, 'MQTT')
            % Get New Data from MQTT under topic: obj.sub_topic
                new_data = obj.mqtt_interface.receive_json(obj.sub_topic); % new data shape (size(uncertainty_grid,1),2n+m): [x_i, x_dot_i, u_i]
            elseif strcmp(obj.comm_mode, 'FileSharing')
                try 
                    new_data = load('data.mat');
                    new_data = new_data.temp;
                    delete('data.mat')
                catch e
                    e
                    new_data = [];
                end
            end
            % new_data = load('data.mat');
            if ~isempty(new_data)
                fprintf('Received new data!\n')
            end
            obj.data(end+1:end+size(new_data,1),:)  = new_data;        % Append new data to stored data
            % Add Fake Disturbance if this is a simulation
            obj.fake_noise = cat(1, obj.fake_noise, normrnd(0,0.05*obj.is_sim,[size(new_data,1),obj.n*obj.m]));
            obj.num_new_data = obj.num_new_data + size(new_data,1);    % count how much new data we've gathered so far
        end
    end
end

