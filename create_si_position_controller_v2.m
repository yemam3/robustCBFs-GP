%% create_si_position_controller 
% Returns a controller ($u: \mathbf{R}^{2 \times N} \times \mathbf{R}^{2 \times N} \to \mathbf{R}^{2 \times N}$) 
% for a single-integrator system.
%% Detailed Description 
% * XVelocityGain - affects the horizontal velocity of the
% single integrator
% * YVelocityGain - affects the vertical velocity of the single integrator
%% Example Usage 
%   si_position_controller = create_si_position_controller('XVelocityGain',
%   1, 'YVelocityGain', 1);
%% Implementation
function [si_position_controller] = create_si_position_controller_v2(varargin)
    
    parser = inputParser;
    addOptional(parser, 'XVelocityGain', 2);
    addOptional(parser, 'YVelocityGain', 2);
    addOptional(parser, 'AngVelocityGain', 4);
    addOptional(parser, 'VelocityMagnitudeLimit', 0.4);
    
    parse(parser, varargin{:});
    
    x_vel_gain = parser.Results.XVelocityGain;
    y_vel_gain = parser.Results.YVelocityGain;
    ang_vel_gain = parser.Results.AngVelocityGain;
    velocity_magnitude_limit = parser.Results.VelocityMagnitudeLimit;
    gains = diag([x_vel_gain ; y_vel_gain]);    
    si_position_controller = @position_si;
    

    function [ dxu ] = position_si(targets, x)
    %POSITIONINT Position controller via single integrator dynamics
    
        % Error checking
        si_to_uni = create_si_to_uni_mapping();
        [M, N] = size(x);
        [M_targets, N_targets] = size(targets); 
        
        assert(M == 3, 'Row size of states (%i) must be 3', M); 
        assert(M_targets== 3, 'Row size of targets (%i) must be 3', M_targets);
        assert(N==N_targets, 'Column size of x (%i) must be the same as targets (%i)', N, N_targets);
        
        dx = gains*(targets(1:2,:) - x(1:2,:));
        dxu = si_to_uni(dx, x);
        
        dxu(abs(dxu)<0.03) = sign(dxu(abs(dxu)<0.03)) * 0.1; % Need to make sure robots are always moving
        
    end
end

