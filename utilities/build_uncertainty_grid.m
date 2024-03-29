function uncertainty_grid = build_uncertainty_grid(bds, varargin)
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
    addOptional(p, 'InitSigma', 50);
    addOptional(p, 'Granularity', 0.25);
    parse(p, varargin{:});
    granularity             = p.Results.Granularity;
    init_sigma              = p.Results.InitSigma;
    % Generate Uncertainty Grid / Heatmap
    [xs, ys, thetas]        = build_grid(bds, 'Granularity', granularity);
    xs                      = reshape(xs, [numel(xs), 1]);
    ys                      = reshape(ys, [numel(ys), 1]);
    thetas                  = reshape(ys, [numel(thetas), 1]);
    sigmas                  = ones(numel(xs), 1) * init_sigma + rand(numel(xs), 1) * 10;
    uncertainty_grid        = [xs, ys, thetas, sigmas]; 
end

