function generate_grid_animation(bds, granularity, zs, varargin)
%GENERATE_SIGMA_ANIMATION Generates 3D surface animation for sigmas
%   Used to highlight the spatio-temporal change in sigmas over the state
%   space meshgrid. 

    
    % Parse Arguments
    p                       = inputParser;
    addOptional(p, 'SavePath', 'animation.gif');
    addOptional(p, 'ZTitle', 'z');
    parse(p, varargin{:});
    save_path               = p.Results.SavePath;
    z_title                 = p.Results.ZTitle;

    % Build the same grid used for the zs
    [xs, ys, thetas]        = build_grid(bds, 'Granularity', granularity);
    
    zs                      = reshape(zs, size(xs,1), size(xs,2), []);
    
    h = surf(xs, ys, zs(:,:,1));
    ax = gca;
    ax.FontSize = 30;
    xlabel('x','Interpreter','latex','FontSize', 40);
    ylabel('y','Interpreter','latex','FontSize', 40);
    zlabel(z_title,'Interpreter','latex','FontSize', 40);
    %ax.CameraPosition = [some_x, some_y, something];
    %h.EdgeColor = 'none';
    h.FaceColor = 'interp';
    h.FaceLighting = 'gouraud';
    view(3)
    grid on;
    first = true;
    xlim([-1.6 1.6])
    ylim([-1 1])
    zlim([min(zs,[],'all')  max(zs,[],'all')])
    set(gcf, 'Position',  [100, 100, 1500, 1000]);
    
    for i = 1:size(zs, 3)
        h.ZData = zs(:,:,i);
        drawnow
        im = getframe(gcf);
        [A,map] = rgb2ind(im.cdata,256);
        if first
            first = false;
            imwrite(A,map, save_path,'LoopCount',Inf,'DelayTime',1/24);
        else
            imwrite(A,map, save_path,'WriteMode','append','DelayTime',1/24);
        end
    end
    
end

