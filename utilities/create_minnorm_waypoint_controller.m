function [ automatic_parking_controller ] = create_minnorm_waypoint_controller(varargin)
% CREATE_WAYPOINT_CONTROLLER Creates a controller that drive a
% unicycle-modeled sytsem to a particular point and stops it (within
% tolerances)
% Works by driving the unicycle to within PositionError of the point then
% rotating it to within RotationError of the desired rotation
%
%   Args:
%       LinearVelocityGain, optional: see also
%       AngularVelocityLimit, optional: see also
%       PositionError, optional: Error tolerance for position
%       PositionEpsilon, optional: Epsilon
%       RotationError, optional: Error tolerance for rotation
%       VelocityMagnitudeLimit, optional: Limit for velocity while driving
%       to position

    p = inputParser;
    addOptional(p, 'LinearVelocityGain', 0.8);
    addOptional(p, 'AngularVelocityLimit', pi/2);
    addOptional(p, 'PositionError', 0.03);
    addOptional(p, 'PositionEpsilon', 0.01)
    addOptional(p, 'RotationError', 0.05);
    addOptional(p, 'VelocityMagnitudeLimit', 0.15)
    parse(p, varargin{:});
    
    lin_vel_gain = p.Results.LinearVelocityGain; 
    ang_vel_limit = p.Results.AngularVelocityLimit;
    vel_mag_limit = p.Results.VelocityMagnitudeLimit;
    pos_err = p.Results.PositionError;
    pos_eps = p.Results.PositionEpsilon;
    rot_err = p.Results.RotationError;
    min_norm = 0.1;
    position_controller = create_si_to_uni_dynamics('LinearVelocityGain', lin_vel_gain, ...
    'AngularVelocityLimit', ang_vel_limit);

    automatic_parking_controller = @automatic_parking_controller_;

    function [dxu] = automatic_parking_controller_(poses, states)
        
        N               = size(states, 2);
        dxu             = zeros(2, N);
        
        % Get wrapped theta difference 
        wrapped         = poses(3, :) - states(3, :);
        wrapped         = atan2(sin(wrapped), cos(wrapped));
        
        % Get si position difference and distances
        diff_si         = poses(1:2, :) - states(1:2, :);
        norm_si         = sqrt(sum(diff_si.^2,1));
        
        % Robots that are still going to waypoints
        lin_ids         = (norm_si > pos_err - pos_eps);
        if any(lin_ids)
            dxi             = diff_si(:,lin_ids);
            norm_si         = norm_si(lin_ids);
            if any(norm_si > vel_mag_limit)
                dxi(:,norm_si > vel_mag_limit) = vel_mag_limit .* dxi(:,norm_si > vel_mag_limit) ./ norm_si(norm_si > vel_mag_limit);
            end
            dxu(:, lin_ids) = position_controller(dxi, states(:, lin_ids));
        end
        
        % Robots that need to rotate to match theta from pose
        rot_ids         = ~lin_ids & (abs(wrapped) > rot_err);
        dxu(1, rot_ids) = 0;
        dxu(2, rot_ids) = sign(wrapped(rot_ids)) .* min(abs(0.5*wrapped(rot_ids)), ang_vel_limit);

        % Max Sure that robots move with a min norm speed
        dxu_norms                       = sqrt(sum(dxu.^2,1));
        if any(dxu_norms<min_norm)
            dxu(:,dxu_norms<min_norm)   = dxu(:,dxu_norms<min_norm) ./ dxu_norms(dxu_norms<min_norm) * min_norm; 
            dxu(isnan(dxu))             = 0; % Takes care of the case where (norm == 0)
        end
        % In case any entry is too close to 0 bump it a bit to ensure
        % robots still move (datapoints are only good if dxu > 0)
        dxu(abs(dxu)<0.05) = sign(dxu(abs(dxu)<0.05)) * 0.05;
        
    end    
end


