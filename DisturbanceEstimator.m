classdef DisturbanceEstimator
    %DISTURBANCEESTIMATOR Estimates disturbance for control affine systems
    %   of the form: x_dot(t) = (g(x(t)) + D(x(t)))u(t)
    %   using Gaussian Processes.
    
    properties (Constant)
        % Transformation Matrix from Differential Drive to Unicycle
        r                   = 0.016;
        l_b                 = 0.105;
        G                   = [0.016/2, 0.016/2; -0.016/0.105, 0.016/0.105];
        L                   = [1, 0; 0, 0.03];
        bds                 = [-1.4, 1.4, -0.8, 0.8]; 
        dt                  = 0.033;            % Robotarium Timestep
        granul_htmp         = 0.15;             % granularity of heatmap
        threshold_data_num = 10;
    end
    properties
        N
        n
        m
        waypoints
        data                    % 3 x num_data_points (variable)
        gpr_models              % cell(n x m)
        uncertainty_grid
        buffer                  % buffer value to stay away from state space border
        max_sigmas              % Record all the max sigmas to plot over time
        num_new_data            % length of new data
    end
    
    methods
        function obj = DisturbanceEstimator(N, n, m)
            %DisturbanceEstimator Construct an instance of this class
            obj.N                   = N;  
            obj.n                   = n;
            obj.m                   = m;
            obj.waypoints           = [];
            obj.data                = zeros([0,2*obj.n+obj.m]);
            obj.uncertainty_grid    = []; % Initialize Big 
            obj.max_sigmas          = [];
            obj.num_new_data        = 0;
            % Intiailize Coordinates for uncertainty grid
            xs                      = obj.bds(1):obj.granul_htmp:obj.bds(2); 
            ys                      = obj.bds(3):obj.granul_htmp:obj.bds(4);
            [xs, ys]                = meshgrid(xs,ys);
            xs                      = reshape(xs, [numel(xs), 1]);
            ys                      = reshape(ys, [numel(ys), 1]);
            thetas                  = 2 * pi * (rand(numel(xs), 1) - 0.5);
            sigmas                  = ones(numel(xs), 1) * 1000;
            obj.uncertainty_grid    = [xs, ys, thetas, sigmas]; 
            % Shape of uncertainty grid:
            % [x_1,y_1,theta_1, sigma_1
            %  x_2,y_2,theta_2, sigma_2  
            %        ....              ]  
            % where x_i, y_i, theta_i is the state, and sigma is the
            % uncertainty at that corresponding point.
        end
        
        function obj = waypoint_step(obj, x)
            %waypoint_step Checks which robot reached its waypoint. And
            %creates new waypoints for the robots which did.
            %   Inputs:
            %       x: 3xN matrix containing state of robots
            %
            if isempty(obj.waypoints)
                ids                 = 1:obj.N;
            else
                ids                 = find(sum((x(1:2,:) - obj.waypoints).^2, 1) < 0.05);
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
            if isempty(obj.uncertainty_grid)
                p                       = rand([2,N_free]) - 0.5;
                p(1,:)                  = (diff(obj.bds(1:2))) * p(1,:);   % Stretch x
                p(2,:)                  = (diff(obj.bds(3:4))) * p(2,:);   % Stretch y
            else
                p                       = obj.uncertainty_grid(1:N_free, 1:2)';
                obj.uncertainty_grid(1:N_free) = [];
            end
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
            y_11                            = x_dot(:,1) ./ u(:,1);
            obj.gpr_models{1,1}             = fitrgp(x, y_11); 
            % g(x)_21
            y_21                            = x_dot(:,2) ./ u(:,1);
            obj.gpr_models{2,1}             = fitrgp(x, y_21); 
            % g(x)_32
            y_32                            = x_dot(:,3) ./ u(:,2);
            obj.gpr_models{3,2}             = fitrgp(x, y_32); 
            % Recompute Uncertainty Grid Based on  
            obj = update_heat_map(obj);
            obj.num_new_data = 0;
        end
        
        function obj = clear_traj_data(obj)
            obj.data  = zeros([0,2*obj.n+obj.m]);
        end
        
        function obj = append_traj_data(obj, x, u, x_old, dxu_old)
            
            if size(obj.data,1) < 100
                ids = 1:obj.N;
            else
                ids = find(sum((x(1:2,:) - obj.waypoints).^2, 1) < 0.05);
            end
            
            x_dot = x(:,ids) - x_old(:,ids);
            x_dot(3,:) = atan2(sin(x_dot(3,:)), cos(x_dot(3,:)));
            x_dot = x_dot / obj.dt;
            u     = dxu_old(:,ids);
            new_data = [x(:,ids); x_dot; u]'; % new data shape: x x_dot u
            new_data(any(abs(u) < 0.001, 1)',:) = []; % Prune data with 0 u
            obj.data(end+1:end+size(new_data,1),:) = new_data;
            obj.num_new_data = obj.num_new_data + size(new_data,1);
        end
        
        function obj = update_heat_map(obj)
            % Clear Sigmas 
            obj.uncertainty_grid(:,4) = zeros(size(obj.uncertainty_grid, 1), 1);
            % Compute Sigma for All points on the grid and sum over GPRs
            for i = 1:obj.n
                for j = 1:obj.m
                    if ~isempty(obj.gpr_models{i,j})
                        [~, sigmas_] = obj.gpr_models{i,j}.predict(obj.uncertainty_grid(:,1:3));
                        obj.uncertainty_grid(:,4) = obj.uncertainty_grid(:,4)  + (sigmas_);
                        [~, ind] = max(sigmas_);
                        fprintf('\tMax sigma_ [%.3f,%.3f,%.3f,%.3f]\n', [obj.uncertainty_grid(ind,1:3),sigmas_(ind)]);
                    end
                end
            end
            [~, inds] = sort(-obj.uncertainty_grid(:,4));
            obj.uncertainty_grid = obj.uncertainty_grid(inds, :);
            obj.max_sigmas(end+1) = obj.uncertainty_grid(1,end);
            fprintf('Max sigma [%.3f,%.3f,%.3f,%.3f]\n', obj.uncertainty_grid(1,:));
        end
        
        function plot_max_sigmas(obj)
            figure(1000)
            plot(obj.max_sigmas);
        end
    end
end

