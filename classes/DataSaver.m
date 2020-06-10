classdef DataSaver
    %DATASAVER Saves Data from Robotarium Experiment
    
    properties
        N
        nominal_radius
        x_old 
        dxu_old
        x_data
        u_data
        u_nom_data
        t_data
        min_h_data
        dt_cbf_data
    end
    
    methods
        function obj = DataSaver(N, nominal_radius)
            %DATASAVER Construct an instance of this class
            obj.N                       = N;
            obj.nominal_radius          = nominal_radius;
            obj.x_old                   = []; 
            obj.dxu_old                 = [];
            obj.x_data                  = zeros(3,N,0); 
            obj.u_data                  = zeros(2,N,0);
            obj.u_nom_data              = zeros(2,N,0);
            obj.t_data                  = [];
            obj.min_h_data              = [];
            obj.dt_cbf_data             = [];
        end
        
        function obj = save(obj, x, dxu, t, dxu_nom, min_h, dt_cbf)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.x_old           = x;
            obj.dxu_old         = dxu;
            obj.x_data          = cat(3,obj.x_data, x);
            obj.u_data          = cat(3,obj.u_data, dxu);
            obj.t_data          = t;
            obj.u_nom_data      = cat(3, obj.u_nom_data, dxu_nom);
            obj.min_h_data      = cat(1, obj.min_h_data, min_h);
            obj.dt_cbf_data     = cat(1, obj.dt_cbf_data, dt_cbf);
        end
        
        function plot_min_h(obj, save_path)
            % Plot min h_ij over time
            iterations = numel(obj.min_h_data);
            fig = figure(1001);hold on;grid on;
            plot(1:iterations, obj.min_h_data, 'LineWidth', 4); 
            plot(1:iterations,zeros(1,iterations),'r', 'LineWidth', 4)
            ylim([-0.1,0.6])
            ax = gca;
            ax.FontSize = 40;
            xlabel('iter', 'FontSize', 50); ylabel('min_{ij}(h_ij(t))', 'FontSize', 50);
            savefig(fig, [save_path, 'min_h_plot.fig']);
        end
        
        function u_diff = plot_u_diff(obj, save_path)
            % Plot difference between u_nom and u over time
            u_diff = squeeze(sum(sum((obj.u_data - obj.u_nom_data).^2, 1),2));
            iterations = numel(u_diff);
            fig = figure(1002);hold on;grid on;
            plot(1:iterations, u_diff, 'LineWidth', 4); 
            ax = gca;
            ax.FontSize = 40;
            xlabel('iter', 'FontSize', 50); ylabel('(u - u_{nom})^2', 'FontSize', 50);
            savefig(fig, [save_path, 'u_diff_plot.fig']);
        end    
    end
end

