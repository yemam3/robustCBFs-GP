files = dir('disturb_estim_data/*.mat');
set(groot,'defaultLineLineWidth',4.0)

dt          = 0.033; 
psi         = 5;
err         = [];
u_wheels_sim = zeros([2,0]); 
u_wheels_act = zeros([2,0]); 
x_dot_si    = zeros([2, 0]); 
x_dot_si_sim    = zeros([2, 0]); 
poses = zeros([3,0]);
%% Extract Trajectory Data
for i = 1:numel(files)
    if(strcmp(files(i).name(1:11), 'disturbance'))
        load(files(i).name);
        % Instantiate Trajectory Processor
        trajProcessor       = TrajectoryProcessor(pose_data, input_data, time, 'dt', dt);  
        % Obtain x_dot_si
        x_dot_si_           = trajProcessor.get_si_vels();                                  % n x N x iters-1
        x_dot_si_           = trajProcessor.flatten_third_dim(x_dot_si_);                   % n x (N*iters-1)
        x_dot_si            = cat(2, x_dot_si, x_dot_si_);                                  % n x (N*iters-1)
        % Obtain x_dot_si_sim
        x_dot_si_sim_       = trajProcessor.get_si_vels_sim();                              % n x N x iters-1
        x_dot_si_sim_       = trajProcessor.flatten_third_dim(x_dot_si_sim_);               % n x (N*iters-1)
        x_dot_si_sim        = cat(2, x_dot_si_sim, x_dot_si_sim_);                          % n x (N*iters-1)        
        % Here we obtain the desired and real input to the wheels
        u_wheels_sim_       = trajProcessor.get_wheel_vels_sim();
        u_wheels_sim_       = trajProcessor.flatten_third_dim(u_wheels_sim_);
        u_wheels_sim        = cat(2, u_wheels_sim, u_wheels_sim_);                          % m x 1 x iters-1
        u_wheels_act_       = trajProcessor.get_wheel_vels();                               
        u_wheels_act_       = trajProcessor.flatten_third_dim(u_wheels_act_);
        u_wheels_act        = cat(2, u_wheels_act, u_wheels_act_);                          % m x 1 x iters-1
        % Obtain Poses 
        poses_              = trajProcessor.flatten_third_dim(trajProcessor.x(:,:,2:end));   % n x (N*iters-1)
        poses               = cat(2, poses, poses_);                                        % n x (N*iters-1)
    end
end
%% Fit Gaussian Process Model
% Setup Data 
labels                  = transpose(x_dot_si ./ x_dot_si_sim);                              % Labels are D(x) (N*iters-1) x n
X                       = transpose(poses);                                                 % size(X) = N * (iter-1)  x n  
bad_lbls                = any(abs(x_dot_si) < 1e-3, 1) | any(abs(x_dot_si_sim) < 1e-3, 1);
labels(bad_lbls, :)     = [];
X(bad_lbls, :)          = [];
inds                    = randperm(size(X, 1));                                             % Inds to Shuffle Data
X                       = X(inds, :);
for i = 1:size(labels, 2)
    y{i}                = labels(inds, i);
end
% Split Training & Testing
split                   = int64(0.8*size(X,1));                                         % Train Size
X_train                 = X(1:split,:);
X_test                	= X(split+1:end,:);
%% Train Gaussian Process Regression (GPR) Models
for i = 1:length(y)
    gprMdl{i}               = fitrgp(X_train, y{i}(1:split)); % y_train 
    ypred_train{i}          = predict(gprMdl{i}, X_train);
    ypred_test{i}           = predict(gprMdl{i}, X_test);
end
%% Plot Data
figure(1)
scatter(y{1}(split+1:end), y{2}(split+1:end))
hold on
scatter(ypred_test{1}, ypred_test{2})
figure(2)
scatter(y{1}(1:split), y{2}(1:split))
hold on
scatter(ypred_train{1}, ypred_train{2})

% fprintf("y1:\t mu = \t%.4f \t Var = \t%.4f\n", mean(y_test{1}), var(y_test{1}))
% fprintf("y2:\t mu = \t%.4f \t Var = \t%.4f\n", mean(y_test{2}), var(y_test{2}))
% fprintf("ypred1:\t mu = \t%.4f \t Var = \t%.4f\n", mean(ypred_test{1}), var(ypred_test{1}))
% fprintf("ypred2:\t mu = \t%.4f \t Var = \t%.4f\n", mean(ypred_test{2}), var(ypred_test{2}))