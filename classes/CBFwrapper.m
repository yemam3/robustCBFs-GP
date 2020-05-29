classdef CBFwrapper
    %CBFwrapper Barrier Function Rapper
    
    properties
        N
        n
        m
        cbf_mode
        cbf_handle
        safety_radius
        projection_distance
        barrier_gain
    end
    
    methods
        
        function obj = CBFwrapper(N, n, m, cbf_specs)
            %CBFwrapper Construct an instance of this class
            obj.N = N; 
            obj.m = m; 
            obj.n = n;
            obj.cbf_mode = cbf_specs.cbf_mode;
            obj.safety_radius = cbf_specs.safety_radius;
            obj.projection_distance = cbf_specs.projection_distance;
            obj.barrier_gain = cbf_specs.barrier_gain;
            % Intialize Barriers and Safety Functions
            if strcmp(obj.cbf_mode, 'Multiplicative')
                obj.cbf_handle = create_uni_barrier_certificate_with_boundary_mult('SafetyRadius', obj.safety_radius, 'BarrierGain', obj.barrier_gain, 'ProjectionDistance', obj.projection_distance);
            elseif strcmp(obj.cbf_mode, 'Additive')
                obj.cbf_handle = create_uni_barrier_certificate_with_boundary_add('SafetyRadius', obj.safety_radius, 'BarrierGain', obj.barrier_gain, 'ProjectionDistance', obj.projection_distance);
            elseif strcmp(obj.cbf_mode, 'Regular')
                obj.cbf_handle = create_uni_barrier_certificate_with_boundary_reg('Disturbance',0,'SafetyRadius', obj.safety_radius, 'BarrierGain', obj.barrier_gain, 'ProjectionDistance', obj.projection_distance);
            else 
                error('CBF_MODE needs to be either Multiplicative or Additive! Check init file.')
            end
        end
        
        function [dxu, min_h] = uni_barrier_certificate(obj, dxu, x, obstacles, mus_, sigmas_)
            %UNI_BARRIER_CERTIFICATE Runs Robust CBFs depending on the
            %declared type.
            
            kd = 2;
            
            if strcmp(obj.cbf_mode, 'Multiplicative')
                mus_            = reshape(mus_',[obj.n,obj.m,obj.N]);
                sigmas_         = reshape(sigmas_',[obj.n,obj.m,obj.N]);
                sigmas_(1,1,:)  = kd*sigmas_(1,1,:);
                sigmas_(2,1,:)  = kd*sigmas_(2,1,:);
                sigmas_(3,2,:)  = kd*sigmas_(3,2,:);
                [dxu, min_h]    = obj.cbf_handle(dxu, x, obstacles, mus_ - sigmas_, mus_ + sigmas_);
            elseif strcmp(obj.cbf_mode, 'Additive')
                mus_            = reshape(mus_',[obj.n,obj.N]);
                sigmas_         = reshape(sigmas_',[obj.n,obj.N]);
                [dxu, min_h]    = obj.cbf_handle(dxu, x, obstacles, mus_ - kd*sigmas_, mus_ + kd*sigmas_); 
            else % No Disturbance
                [dxu, min_h]    = obj.cbf_handle(dxu, x, obstacles);
            end
        end
        
    end
    
end

