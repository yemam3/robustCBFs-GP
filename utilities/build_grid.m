function [xs, ys, thetas] = build_grid(bds, varargin)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
% Shape of uncertainty grid:
% [x_1,y_1,theta_1, sigma_1
%  x_2,y_2,theta_2, sigma_2  
%        ....              ]  
% where x_i, y_i, theta_i is the state, and sigma is the
% uncertainty at that corresponding point.
    
    % Parse Arguments
    p = inputParser;
    addOptional(p, 'Granularity', 0.25);
    parse(p, varargin{:});
    granularity             = p.Results.Granularity;
    % Generate Uncertainty Grid / Heatmap
    xs                      = bds(1):granularity:bds(2);
    ys                      = bds(3):granularity:bds(4);
    [xs, ys]                = meshgrid(xs,ys);
    thetas                  = 2 * pi * (rand(size(xs)) - 0.5);
end

