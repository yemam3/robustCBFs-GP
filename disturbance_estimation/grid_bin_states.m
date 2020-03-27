function [bins_id] = grid_bin_states(x)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

grid_size = 0.05;
bds       = [-1.6, 1.6, -1.0, 1.0]; % State Space Bounds 1 x (n*2)

assert(size(x, 1) == length(bds)/2, 'Grid binning requires only x,y.');

% Shifts states to lower-bound them by 0
x = x - bds(1:2:end)';
% Bin Each Axis
bins_id = floor(x / grid_size);

end

