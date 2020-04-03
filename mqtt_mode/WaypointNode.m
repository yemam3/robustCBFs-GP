classdef WaypointNode
    %DISTURBANCEESTIMATOR Estimates disturbance for control affine systems
    %   of the form: x_dot(t) = (g(x(t)) + D(x(t)))u(t)
    %   using Gaussian Processes.
    
    properties (Constant)
        % Transformation Matrix from Differential Drive to Unicycle
        pub_topic           = 'data';
        sub_topic           = 'sigmas';
        bds                 = [-1.4, 1.4, -0.8, 0.8]; 
        dt                  = 0.033;            % Robotarium Timestep
        granul_htmp         = 0.25;             % granularity of heatmap
    end
    properties
        mqtt_interface
        N
        n
        m
        waypoints
        waypoint_queue
        data                    % 3 x num_data_points (variable)
        uncertainty_grid
        max_sigmas              % Record all the max sigmas to plot over time
        all_sigmas
    end
    
    methods
        function obj = WaypointNode(N, n, m)
            %DisturbanceEstimator Construct an instance of this class
            % Setup MQTT Node
            % Robotarium: mqttInterface = MqttInterface('matlab_node', '192.168.1.8', 1884); 
            % Localhost:  mqttInterface = MqttInterface('matlab_node', 'localhost', 1883); 
            obj.mqtt_interface       = MqttInterface('waypoint_node', 'localhost', 1883, 1);
            obj.mqtt_interface.subscribe(obj.sub_topic);
            obj.N                   = N;  
            obj.n                   = n;
            obj.m                   = m;
            obj.waypoints           = [];
            obj.waypoint_queue      = [];
            obj.data                = zeros([0,2*obj.n+obj.m]);
            % For Saving Purposes 
            obj.all_sigmas          = [];
            % Intiailize Coordinates for uncertainty grid
            obj.uncertainty_grid    = build_uncertainty_grid(obj.bds,obj.granul_htmp);    
        end
        
        function obj = waypoint_step(obj, x)
            %waypoint_step Checks which robot reached its waypoint. And
            %creates new waypoints for the robots which did.
            %   Inputs:
            %       x: 3xN matrix containing state of robots
            %
            obj                     = obj.update_heatmap();
            if isempty(obj.waypoints)
                ids                 = 1:obj.N;
            else
                ids1                = sum((x(1:2,:) - obj.waypoints(1:2,:)).^2, 1) < 0.15;
                theta_diff          = (x(3,:) - obj.waypoints(3,:));
                ids2                = abs(atan2(sin(theta_diff), cos(theta_diff))) < pi/6;
                ids                 = find(ids1 & ids2);
            end
            obj.waypoints(:,ids)    = obj.gen_next_waypoint(x, ids);
        end
        
        function p = gen_next_waypoint(obj, x, ids)
            %gen_next_waypoint Generates next waypoints for each robot.
            %   Inputs:
            %       x: 3xN matrix containing state of robots
            %     ids: 1xN_free vector containing inds of free robots
            %   Outputs:
            %       p: 2xN matrix containing next waypoints
            %
            N_free                  = length(ids);
            if isempty(obj.waypoint_queue)
                p                       = rand([3,N_free]) - 0.5;
                p(1,:)                  = (diff(obj.bds(1:2))) * p(1,:);   % Stretch x
                p(2,:)                  = (diff(obj.bds(3:4))) * p(2,:);   % Stretch y
                p(3,:)                  = 2 * pi * p(3,:);                 % Stretch theta [-pi, pi]
            else
                p                       = obj.waypoint_queue(1:N_free, 1:obj.n)';
                
                obj.waypoint_queue(1:N_free) = [];
            end
        end
        
        function obj = clear_traj_data(obj)
            obj.data  = zeros([0,2*obj.n+obj.m]);
        end
        
        function obj = append_traj_data(obj, x, dxu, x_old, dxu_old)
            % Get New Data from MQTT under topic: obj.sub_topic
            ids                                     = 1:obj.N;                      % Going to always collect data instead 
            x_dot                                   = x(:,ids) - x_old(:,ids);
            x_dot(3,:)                              = atan2(sin(x_dot(3,:)), cos(x_dot(3,:)));
            x_dot                                   = x_dot / obj.dt * 1.1;%+ normrnd(0,1.00); % Add Fake Noise Here
            u                                       = dxu_old(:,ids);
            new_data                                = [x_old(:,ids); x_dot; u]';    % new data shape: x x_dot u
            new_data(any(abs(u) < 0.001, 1)',:)     = [];                           % Prune data with 0 u
            obj.data(end+1:end+size(new_data,1),:)  = new_data;
            % Send then Clear Newly Collected Data Points 
            if size(obj.data,1) > 10
                obj.mqtt_interface.send_json(obj.pub_topic, obj.data);
                obj.clear_traj_data();       
            end
        end
        
        function obj = update_heatmap(obj)
            sigmas = obj.mqtt_interface.receive_json(obj.sub_topic); % new data shape (-1,2n+m): [x_i, x_dot_i, u_i]
            if isempty(sigmas) 
                return
            end
            obj.all_sigmas = [obj.all_sigmas; sigmas'];
            %fprintf('Received New Sigmas\n');
            % Update Sigmas
            obj.uncertainty_grid(:,4) = sigmas;
            % Update Waypoint Queue
            [~, inds]               = sort(-obj.uncertainty_grid(:,4));
            obj.waypoint_queue      = obj.uncertainty_grid(inds, :);
            obj.max_sigmas(end+1)   = obj.waypoint_queue(1,4);
            % Remove Points the robots are currently going to
            for i = 1:obj.N
               obj.waypoint_queue(all(obj.waypoint_queue(:,1:3) == obj.waypoints(:,i)',2),:) = [];
            end
        end
     
    end
end

