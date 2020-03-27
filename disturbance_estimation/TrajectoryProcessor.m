classdef TrajectoryProcessor
    %TRAJECTORYPROCESSOR Given poses and inputs, return disturbance
    %data for GRITSBotsX. 
    
   properties (Constant)
        % Transformation Matrix from Differential Drive to Unicycle
        r           = 0.016;
        l_b         = 0.105;
        G           = [0.016/2, 0.016/2; -0.016/0.105, 0.016/0.105];
        L           = [1, 0; 0, 0.03];
   end
    properties
        x 
        u
        live_t
        N
        n
        m
        iters
        hFig
        dt          
    end
    methods
        function obj = TrajectoryProcessor(poses, inputs, time, varargin)
            %TRAJECTORYPROCESSOR Constructor 
            ip = inputParser;
            addParameter(ip, 'robotType', 'diffDrive')
            addParameter(ip, 'figureHandle', [])
            addParameter(ip, 'dt', 0.033)
            parse(ip,varargin{:})
            % Trajectory To Be Stored
            obj.x           = poses;            % 3 x N x iters
            obj.u           = inputs;           % 2 x N x iters
            obj.live_t      = time;             % 1 x iters
            obj.N           = size(poses, 2);
            obj.n           = size(poses, 1);
            obj.m           = size(inputs, 1);
            obj.iters       = size(poses, 3);
            % Internal Parameters
            obj.dt          = ip.Results.dt;
            % Figure Handle
            obj.hFig        = ip.Results.figureHandle;
        end
        
        function x_dot_si = get_si_vels(obj, varargin)
            %GET_SI_VELS Returns the ACTUAL or SIMULATED single integrator velocities of all robots.
            % return x_dot_sim_si: 2 x N x floor((iters-1)/ss)

            ip = inputParser;
            addParameter(ip, 'sub_sample', 1)
            addParameter(ip, 'is_sim', -1);
            parse(ip,varargin{:})

            ss = ip.Results.sub_sample;                                  % Subsampling rate
            is_sim = ip.Results.is_sim; 
            
            assert(is_sim ~= -1, 'Please specify whether you want the actual or simulated x_dot_si');
            
            if is_sim % Simulated x_dot_si
                x_dot_si = zeros(2, obj.N, floor((obj.iters-1)/ss));
                for t = floor(ss/2):ss:obj.iters
                    % Terminating condition (TODO: can be improved)
                    if ceil(t/ss) > size(x_dot_si, 3)
                        break
                    end
                    for i = 1:obj.N 
                        x_dot_si(:, i, ceil(t/ss)) = [cos(obj.x(3,i,t)) -sin(obj.x(3,i,t)); sin(obj.x(3,i,t)) cos(obj.x(3,i,t))] * obj.L * obj.u(:,i,t);
                    end 
                end                
            else      % Actual x_dot_si
                x_dot_si       = diff(obj.x(1:2,:,1:ss:end), 1, 3)./(obj.dt*ss); % 2 x N x (iters//ss -1)
            end
        end
        
        function x_dot_uni = get_uni_vels(obj, varargin)
            %GET_UNI_VELS Returns the ACTUAL or SIMULATED unicycle velocities of all robots.
            
            ip = inputParser;
            addParameter(ip, 'sub_sample', 1)
            addParameter(ip, 'is_sim', -1);
            parse(ip,varargin{:})
            ss = ip.Results.sub_sample;                                  % Subsampling rate
            is_sim = ip.Results.is_sim; 
            assert(is_sim ~= -1, 'Please specify whether you want the actual or simulated x_dot_si');

            if is_sim
                x_dot_uni               = obj.u(:,:,1:ss:floor((obj.iters-1)/ss));          % 2 x N x (iters-1) // ss
            else
                x_dot_si                = get_si_vels(obj, 'sub_sample', ss, 'is_sim', is_sim);
                % Calculate Linear Velocity using x_dot_si
                x_dot_uni               = zeros(size(x_dot_si));        % 2 x N x (iters-1) // ss
                x_dot_uni(1,:,:)        = sqrt(sum(x_dot_si.^2, 1));  
                % Calculate Angular Velocity using x_dot_si
                theta_diffs             = diff(obj.x(3, :, 1:ss:end), 1, 3);
                theta_diffs             = atan2(sin(theta_diffs), cos(theta_diffs));
                x_dot_uni(2,:,:)        = theta_diffs./obj. dt;   
            end
            
        end
        
        function x_dot_dd = get_dd_vels(obj, varargin)
            %GET_DD_VELS Returns the ACTUAL or Simulated wheel velocities of all robots.
            
            % Parse Inputs
            ip              = inputParser;
            addParameter(ip, 'sub_sample', 1)
            addParameter(ip, 'is_sim', -1);
            parse(ip,varargin{:})
            ss              = ip.Results.sub_sample;                                                    % Subsampling rate
            is_sim          = ip.Results.is_sim; 
            % Make Sure user specified whether he wants sim or real
            assert(is_sim ~= -1, 'Please specify whether you want the actual or simulated x_dot_si');
            % Get differential drive inputs to the wheels
            x_dot_uni               = obj.get_uni_vels('sub_sample', ss, 'is_sim', is_sim);             % 2 x N x (iters - 1)
            x_dot_dd                = zeros(size(x_dot_uni)); 
            for i = 1:obj.N
                x_dot_dd(:,i,:)     = obj.G \ reshape(x_dot_uni(:,i,:), [2, size(x_dot_dd, 3)]);        % 2 x iters-1
            end
            
        end
                
        function matrix_2D = flatten_third_dim(obj, data)
            % FLATTEN_THIRD_DIM Returns a flattened 3D-matrix.
            % Specifically, it joins the 2nd and 3rd dimension.
            assert(length(size(data)) == 3, "Matrix is not 3D");
            matrix_2D = reshape(data, [size(data, 1), size(data, 2) * size(data, 3)]);
        end
        
    end
end

