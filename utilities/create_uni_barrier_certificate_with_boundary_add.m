function [ uni_barrier_certificate ] = create_uni_barrier_certificate_with_boundary_add(varargin)
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
    addOptional(parser, 'BarrierGain', 150);
    addOptional(parser, 'SafetyRadius', 0.12);
    addOptional(parser, 'ProjectionDistance', 0.03);
    addOptional(parser, 'BaseLength', 0.105);
    addOptional(parser, 'WheelRadius', 0.016);
    addOptional(parser, 'WheelVelocityLimit', 12.5);
    addOptional(parser, 'MaxNumRobots', 30);
    addOptional(parser, 'MaxObstacles', 50);
    addOptional(parser, 'MaxNumBoundaryPoints', 4);
    addOptional(parser, 'BoundaryPoints', [-1.6 1.6 -1.0 1.0]);
    parse(parser, varargin{:})  
    
    opts = optimoptions(@quadprog,'Display', 'off', 'TolFun', 1e-5, 'TolCon', 1e-4);       
    gamma = parser.Results.BarrierGain;
    safety_radius = parser.Results.SafetyRadius;
    projection_distance = parser.Results.ProjectionDistance;
    wheel_radius = parser.Results.WheelRadius;
    base_length = parser.Results.BaseLength;
    wheel_vel_limit = parser.Results.WheelVelocityLimit;
    max_num_robots = parser.Results.MaxNumRobots;
    max_num_boundaries = parser.Results.MaxNumBoundaryPoints;
    max_num_obstacles = parser.Results.MaxObstacles;
    boundary_points = parser.Results.BoundaryPoints;
    
    %Check given boundary points
    assert(length(boundary_points)==4, "Boundary points must represent a rectangle.")
    assert(boundary_points(2) > boundary_points(1), "Difference between x coordinates of defined rectangular boundary points must be positive.")
    assert(boundary_points(4) > boundary_points(3), "Difference between y coordinates of defined rectangular boundary points must be positive.")
     
    D = [wheel_radius/2, wheel_radius/2; -wheel_radius/base_length, wheel_radius/base_length];
    L = [1,0;0,projection_distance] * D;
% 
%     max_num_constraints = (num_disturbs^2)*nchoosek(max_num_robots, 2) + max_num_robots*max_num_obstacles*num_disturbs + max_num_robots;
    
    max_num_constraints = nchoosek(max_num_robots, 2) + max_num_robots*max_num_boundaries + max_num_robots*max_num_obstacles; %+ max_num_robots;
    A = zeros(max_num_constraints, 2*max_num_robots);
    b = zeros(max_num_constraints, 1);

    Os = zeros(2,max_num_robots);
    ps = zeros(2,max_num_robots);
    Ms = zeros(2,2*max_num_robots);
    
   
    
    uni_barrier_certificate = @barrier_unicycle;
    

    function [ dxu, ret ] = barrier_unicycle(dxu, x, obstacles, psi_1, psi_2)   
        % BARRIER_UNICYCLE The parameterized barrier function
        %
        %   Args:
        %       dxu: 2xN vector of unicycle control inputs
        %       x: 3xN vector of unicycle states
        %       obstacles: Optional 2xN vector of obtacle points.
        %
        %   Returns:
        %       A 2xN matrix of safe unicycle control inputs
        %
        %   BARRIER_UNICYCLE(dxu, x)
        
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
        num_constraints = temp + num_robots*num_obstacles + 4*num_robots;
        A(1:num_constraints, 1:2*num_robots) = 0;
        Os(1,1:num_robots) = cos(x(3, :)); 
        Os(2,1:num_robots) = sin(x(3, :));
        ps(:,1:num_robots) = x(1:2, :) + projection_distance*Os(:,1:num_robots);
        Ms(1,1:2:2*num_robots) = Os(1,1:num_robots);
        Ms(1,2:2:2*num_robots) = -projection_distance*Os(2,1:num_robots);
        Ms(2,2:2:2*num_robots) = projection_distance*Os(1,1:num_robots);
        Ms(2,1:2:2*num_robots) = Os(2,1:num_robots);
        ret = zeros(1, temp);
        
        % Sum the convex hulls for p_dot (check transformation)
        psi_1(4,:) = projection_distance * cos(x(3, :)) .* psi_1(3,:);
        psi_1(3,:) = -projection_distance * sin(x(3, :)) .* psi_1(3,:);
        psi_2(4,:) = projection_distance * cos(x(3, :)) .* psi_2(3,:);
        psi_2(3,:) = -projection_distance * sin(x(3, :)) .* psi_2(3,:);
        disturb_1 = psi_1(1:2,:) + min(psi_1(3:4,:),psi_2(3:4,:)); % 2xN
        disturb_2 = psi_2(1:2,:) + max(psi_1(3:4,:),psi_2(3:4,:)); % 2xN
        
        
        count = 1;
        for i = 1:(num_robots-1)
            for j = (i+1):num_robots
                diff = ps(:, i) - ps(:, j);
                hs = sum(diff.^2,1) - safety_radius^2;
                h_dot_i = 2*(diff)'*Ms(:,2*i-1:2*i);
                h_dot_j = -2*(diff)'*Ms(:,2*j-1:2*j);                
                A(count, (2*i-1):(2*i)) = h_dot_i*D;
                A(count, (2*j-1):(2*j)) = h_dot_j*D;
                b(count) = -gamma*hs.^3 - min(h_dot_i*disturb_1(:,i), h_dot_i*disturb_2(:,i)) - min(h_dot_j*disturb_1(:,j), h_dot_j*disturb_2(:,j));  
                ret(count) = hs;
                
                count = count + 1;
            end
        end
        
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
            A(count,(2*i-1):(2*i)) =  -Ms(2,(2*i-1):(2*i))*D;
            b(count) = -0.4*gamma*(boundary_points(4) - safety_radius/2 - ps(2,i))^3;
            count = count + 1;

            %Neg Y
            A(count,(2*i-1):(2*i)) =  Ms(2,(2*i-1):(2*i))*D;
            b(count) = -0.4*gamma*(-boundary_points(3) - safety_radius/2 + ps(2,i))^3;
            count = count + 1;

            %Pos X
            A(count,(2*i-1):(2*i)) =  -Ms(1,(2*i-1):(2*i))*D;
            b(count) = -0.4*gamma*(boundary_points(2) - safety_radius/2 - ps(1,i))^3;
            count = count + 1;

            %Neg X
            A(count,(2*i-1):(2*i)) =  Ms(1,(2*i-1):(2*i))*D;
            b(count) = -0.4*gamma*(-boundary_points(1) - safety_radius/2 + ps(1,i))^3;
            count = count + 1;
        end
        
        %Solve QP program generated earlier
        L_all = kron(eye(num_robots), L);
        dxu = D \ dxu; % Convert user input to differential drive
        %dxu(1:2,1:4)
        vhat = reshape(dxu,2*num_robots,1);
        %disp('vhat')
        %vhat(1:4)
        H = 2*(L_all')*L_all;
        f = -2*vhat'*(L_all')*L_all;
        vnew = quadprog(H, double(f), -A(1:num_constraints,1:2*num_robots), -b(1:num_constraints), [], [], -wheel_vel_limit*ones(2*num_robots,1), wheel_vel_limit*ones(2*num_robots,1), [], opts);
        %disp('f')
        %f(1:4)
        %Set robot velocities to new velocities
        dxu = reshape(vnew, 2, num_robots);
        dxu = D*dxu;
        
    end
end
