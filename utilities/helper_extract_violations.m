%% Helper Script to Excract Barriers Violations from Data
% Need x_data of size (3,N,T) in the workspace

% Safety Radius Assumed for Barriers
safety_radius = 0.12;
projection_distance = 0.03;
counter = 0;
violations = 0;

for t = 1:size(x_data,3)
    x = x_data(:,:,t);
    % Sines and Cosines (size(Os) = (2,N))
    Os = [cos(x(3, :)); sin(x(3, :))]; 
    % Point Look-aheads for the robots
    ps = x(1:2, :) + projection_distance*Os;    
    for i = 1:(num_robots-1)
        for j = (i+1):num_robots
            % Difference between centroids of robots
            diff = ps(:, i) - ps(:, j);
            % h is the barrier function
            hs = sum(diff.^2,1) - safety_radius^2; 
            if hs < 0
                violations = violations + 1;
            end
            counter = counter + 1;
        end
    end
end

fprintf('Fraction of violations = %.3f\n', violations/counter);
