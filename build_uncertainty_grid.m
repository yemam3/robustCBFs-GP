function uncertainty_grid = build_uncertainty_grid(bds,granul_htmp,varargin)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
% Shape of uncertainty grid:
% [x_1,y_1,theta_1, sigma_1
%  x_2,y_2,theta_2, sigma_2  
%        ....              ]  
% where x_i, y_i, theta_i is the state, and sigma is the
% uncertainty at that corresponding point.
    
    if nargin >= 3
        init_sigma           = argin(3);
    else
        init_sigma          = 1000;
    end
    
    xs                      = bds(1):granul_htmp:bds(2);
    ys                      = bds(3):granul_htmp:bds(4);
    [xs, ys]                = meshgrid(xs,ys);
    xs                      = reshape(xs, [numel(xs), 1]);
    ys                      = reshape(ys, [numel(ys), 1]);
    thetas                  = 2 * pi * (rand(numel(xs), 1) - 0.5);
    sigmas                  = ones(numel(xs), 1) * init_sigma;
    uncertainty_grid        = [xs, ys, thetas, sigmas]; 
end

