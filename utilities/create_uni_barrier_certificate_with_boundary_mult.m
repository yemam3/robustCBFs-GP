function [ uni_barrier_certificate ] = create_uni_barrier_certificate_with_boundary_mult(varargin)
% CREATE_SI_BARRIER_CERTIFICATE Creates a unicycle barrier
% certificate function to avoid collisions.
%
%   Args:
%       BarrierGain, optional: How quickly robots can approach eachother
%       SafetyRadius, optional: How far apart centers of robots should
%       remain
%       ProjectionDistance, optional: How far ahead to project a virtual
%       single integrator
%       VelocityMagnitudeLimit, optional: The maximum velocity for the
%       virtual single integrator
%   
%   Returns:
%       A barrier certificate function (2xN, 3xN) -> 2xN representing the
%       barrier certificate
%
%   CREATE_UNI_BARRIER_CERTIFICATE('BarrierGain', bg)
%
%   CREATE_UNI_BARRIER_CERTIFICATE('SafetyRadius', sr)
%
%   CREATE_UNI_BARRIER_CERTIFICATE('SafetyRadius', sr, 'BarrierGain', bg)
%
%   Example:
%       bc = create_si_barrier_certificate('SafetyRadius', 0.2)
%   
%   Notes:
%       SafetyRadius should be a positive double
%       BarrierGain should be a positive double
%       In practice, the value for SafetyRadius should be a little more than double the
%       size of the robots. 

    parser = inputParser;
    addOptional(parser, 'BarrierGain', 500);
    addOptional(parser, 'SafetyRadius', 0.12);
    addOptional(parser, 'ProjectionDistance', 0.03);
    addOptional(parser, 'WheelVelocityLimit', 12.5);
    addOptional(parser, 'BaseLength', 0.105);
    addOptional(parser, 'WheelRadius', 0.016);
    addOptional(parser, 'MaxNumRobots', 10);
    addOptional(parser, 'MaxObstacles', 50);
    addOptional(parser, 'MaxNumBoundaryPoints', 4);
    addOptional(parser, 'BoundaryPoints', [-1.6 1.6 -1.0 1.0]);
    addOptional(parser, 'MaxDisturbance', 10);
    parse(parser, varargin{:})  
    
    opts = optimoptions(@quadprog,'Display', 'off', 'TolFun', 1e-5, 'TolCon', 1e-4);       
    gamma               = parser.Results.BarrierGain;
    safety_radius       = parser.Results.SafetyRadius;
    projection_distance = parser.Results.ProjectionDistance;
    wheel_vel_limit     = parser.Results.WheelVelocityLimit;
    wheel_radius        = parser.Results.WheelRadius;
    base_length         = parser.Results.BaseLength;
    max_num_robots      = parser.Results.MaxNumRobots;
    max_num_boundaries  = parser.Results.MaxNumBoundaryPoints;
    max_num_obstacles   = parser.Results.MaxObstacles;
    boundary_points     = parser.Results.BoundaryPoints;
    max_disturb         = parser.Results.MaxDisturbance;

    %Check given boundary points
    assert(length(boundary_points)==4, "Boundary points must represent a rectangle.")
    assert(boundary_points(2) > boundary_points(1), "Difference between x coordinates of defined rectangular boundary points must be positive.")
    assert(boundary_points(4) > boundary_points(3), "Difference between y coordinates of defined rectangular boundary points must be positive.")
    
    D = [wheel_radius/2, wheel_radius/2; -wheel_radius/base_length, wheel_radius/base_length];
    L = [1,0;0,projection_distance];
    
    
    max_num_constraints = nchoosek(max_num_robots, 2) + max_num_robots*max_num_boundaries + max_num_robots*max_num_obstacles; %+ max_num_robots;
    A = zeros(max_num_constraints, 2*max_num_robots);
    b = zeros(max_num_constraints, 1);

    Os = zeros(2,max_num_robots);
    ps = zeros(2,max_num_robots);
    Ms_1 = zeros(2,2*max_num_robots);
    Ms_2 = zeros(2,2*max_num_robots);
    Ms = zeros(2,2*max_num_robots);
    
    uni_barrier_certificate = @barrier_unicycle;
    

    function [ dxu, ret, dt ] = barrier_unicycle(dxu, x, obstacles, psi_1, psi_2)   
        % BARRIER_UNICYCLE The parameterized barrier function
        %
        %   Args:
        %       dxu: 2xN vector of unicycle control inputs
        %       x: 3xN vector of unicycle states
        %       obstacles: Optional 2xN vector of obtacle points.
        %       psi: Multiplicative disturbance on g(x) (3x2xN)
        %
        %   Returns:
        %       A 2xN matrix of safe unicycle control inputs
        %
        %   BARRIER_UNICYCLE(dxu, x)
        
        % Bound The Disturbance just in case we're fed non-sense
        
        psi_1(abs(psi_1) > max_disturb) = sign(psi_1(abs(psi_1) > max_disturb)) * max_disturb;
        psi_2(abs(psi_2) > max_disturb) = sign(psi_2(abs(psi_2) > max_disturb)) * max_disturb;
        
        if nargin < 3
            obstacles = [];
        end
        
        num_robots = size(dxu, 2);
        num_obstacles = size(obstacles, 2);
        
        if(num_robots < 2)
           temp = 0;
        else
           temp = nchoosek(num_robots, 2); 
        end
           
        %Generate constraints for barrier certificates based on the size of
        %the safety radius
        num_constraints = 16*temp + 4*num_robots*num_obstacles + 16*num_robots;
        
        % Add 1 to all entries of psi_32 
        psi_1(3,2,:) = psi_1(3,2,:) + 1;
        psi_2(3,2,:) = psi_2(3,2,:) + 1;
        % Each robot now has its own 2Ls
        Ls_1 = repmat([1,0;projection_distance,projection_distance], [1,num_robots]);
        Ls_2 = repmat([1,0;projection_distance,projection_distance], [1,num_robots]);
        % Multiply by psi(3,1) (v term for theta_dot) 
        Ls_1(2,1:2:2*num_robots) =  Ls_1(2,1:2:2*num_robots) .* squeeze(psi_1(3,1,:))';
        Ls_2(2,1:2:2*num_robots) =  Ls_2(2,1:2:2*num_robots) .* squeeze(psi_2(3,1,:))';
        % Multiply by psi(3,2) (omega term for theta_dot)
        Ls_1(2,2:2:2*num_robots) =  Ls_1(2,2:2:2*num_robots) .* squeeze(psi_1(3,2,:))';
        Ls_2(2,2:2:2*num_robots) =  Ls_2(2,2:2:2*num_robots) .* squeeze(psi_2(3,2,:))';
        % Sines and Cosines (size(Os) = (2,N))
        Os(1,1:num_robots) = cos(x(3, :)); % 1st row cos_i 
        Os(2,1:num_robots) = sin(x(3, :)); % 2nd row sin_i
        % Rotation Matrices
        Rs = zeros(2,2*num_robots);
        Rs(1,1:2:2*num_robots) =  Os(1,1:num_robots);   % cos_i
        Rs(1,2:2:2*num_robots) =  -Os(2,1:num_robots);  % -sin_i
        Rs(2,1:2:2*num_robots) =  Os(2,1:num_robots);   % sin_i
        Rs(2,2:2:2*num_robots) =  Os(1,1:num_robots);   % cos_i
        % Multiply R by Ls
        for i = 1:num_robots
            Ms_1(:,2*i-1:2*i) = Rs(:,2*i-1:2*i) * Ls_1(:,2*i-1:2*i);
            Ms_2(:,2*i-1:2*i) = Rs(:,2*i-1:2*i) * Ls_2(:,2*i-1:2*i);
        end
        % Need to re-arrange Ms_1 and Ms_2 so that all the mins are in Ms_1
        % and all the maxs are in Ms_2
        Ms_temp_min = min(Ms_1(:,1:2*num_robots), Ms_2(:,1:2*num_robots)) + psi_1(1:2,:);
        Ms_2(:,1:2*num_robots) = max(Ms_1(:,1:2*num_robots), Ms_2(:,1:2*num_robots)) + psi_2(1:2,:);
        Ms_1(:,1:2*num_robots) = Ms_temp_min;
        

        
        A(1:num_constraints, 1:2*num_robots) = 0;
        
        % Point Look-aheads for the robots
        ps(:,1:num_robots) = x(1:2, :) + projection_distance*Os(:,1:num_robots);
        
        % Want to return pairwise hs as well
        ret = 10;
        
        count = 1;
        for i = 1:(num_robots-1)
            for j = (i+1):num_robots
                % Difference between centroids of robots
                diff = ps(:, i) - ps(:, j);
                if sum(diff.^2,1) < ret
                    ret = sum(diff.^2,1);
                end
                % h is the barrier function
                hs = sum(diff.^2,1) - safety_radius^2;
                % We want to multiply diff by M in a way we can add
                % the intervals (need to min, max)
                diff_rep = repmat(diff, [1,2]);
                h_dot_is = 2*[sum(min(diff_rep .* Ms_1(:,2*i-1:2*i), diff_rep .* Ms_2(:,2*i-1:2*i)),1);
                            sum(max(diff_rep .* Ms_1(:,2*i-1:2*i), diff_rep .* Ms_2(:,2*i-1:2*i)),1)];
                h_dot_js = -2*[sum(min(diff_rep .* Ms_1(:,2*j-1:2*j), diff_rep .* Ms_2(:,2*j-1:2*j)),1);
                            sum(max(diff_rep .* Ms_1(:,2*j-1:2*j), diff_rep .* Ms_2(:,2*j-1:2*j)),1)]; 
                % Now we have 16 possible combinations of disturbances
                % possible
                for k1 = 1:2
                    for k2 = 1:2
                        for k3 = 1:2
                            for k4 = 1:2
                                h_dot_i = [h_dot_is(k1,1), h_dot_is(k2,2)] * D;
                                h_dot_j = [h_dot_js(k3,1), h_dot_js(k4,2)] * D;
                                A(count, (2*i-1):(2*i)) = h_dot_i;
                                A(count, (2*j-1):(2*j)) = h_dot_j;
                                b(count) = -gamma*hs.^3; 
                                count = count + 1;
                            end
                        end
                    end
                end
            end
        end
        
        Ms(1,1:2:2*num_robots) = Os(1,1:num_robots);
        Ms(1,2:2:2*num_robots) = -projection_distance*Os(2,1:num_robots);
        Ms(2,2:2:2*num_robots) = projection_distance*Os(1,1:num_robots);
        Ms(2,1:2:2*num_robots) = Os(2,1:num_robots);
        
        if ~isempty(obstacles)
            % Do obstacles
            for i = 1:num_robots            
                diffs = (ps(:, i) - obstacles)';
                h = sum(diffs.^2, 2) - safety_radius^2;
                h_dot_i = 2*diffs*Ms(:,2*i-1:2*i)*D;
                A(count:count+num_obstacles-1,(2*i-1):(2*i)) = h_dot_i;
                b(count:count+num_obstacles-1) = -gamma*h.^3;               
                count = count + num_obstacles;
            end
        end
        
        for i = 1:num_robots
            %Pos Y
            A(count,(2*i-1):(2*i)) =  -Ms(2,(2*i-1):(2*i)) * D;
            b(count) = -0.4*gamma*(boundary_points(4) - safety_radius/2 - ps(2,i))^3;
            count = count + 1;

            %Neg Y
            A(count,(2*i-1):(2*i)) =  Ms(2,(2*i-1):(2*i)) * D;
            b(count) = -0.4*gamma*(-boundary_points(3) - safety_radius/2 + ps(2,i))^3;
            count = count + 1;

            %Pos X
            A(count,(2*i-1):(2*i)) =  -Ms(1,(2*i-1):(2*i)) * D;
            b(count) = -0.4*gamma*(boundary_points(2) - safety_radius/2 - ps(1,i))^3;
            count = count + 1;

            %Neg X
            A(count,(2*i-1):(2*i)) =  Ms(1,(2*i-1):(2*i)) * D;
            b(count) = -0.4*gamma*(-boundary_points(1) - safety_radius/2 + ps(1,i))^3;
            count = count + 1;
        end
        
        %Solve QP program generated earlier
        dxu = D \ dxu;
        vhat = reshape(dxu,2*num_robots,1);
        L_all = kron(eye(num_robots), L*D);
        H = 2*(L_all')*L_all;
        f = -2*vhat'*(L_all')*L_all;
        tic;
        [vnew,FVAL,EXITFLAG,OUTPUT,LAMBDA] = quadprog(H, double(f), -A(1:num_constraints,1:2*num_robots), -b(1:num_constraints), [], [], -wheel_vel_limit*ones(2*num_robots,1), wheel_vel_limit*ones(2*num_robots,1), [], opts);
        dt = toc;
        if isempty(vnew)
            dxu = zeros(2, num_robots);
            warning('No Solution for Barriers!')
        else
            %Set robot velocities to new velocities
            dxu = reshape(vnew, 2, num_robots);
            dxu = D * dxu;
        end
    end
end

