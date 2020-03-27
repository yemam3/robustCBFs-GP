files = dir('disturb_estim_data/*.mat');
set(groot,'defaultLineLineWidth',4.0)

dt                      = 0.033; 
psi                     = 5;
err                     = [];
x_dot_dd                = zeros([2, 0]); 
x_dot_uni               = zeros([2, 0]); 
x_dot_si                = zeros([2, 0]); 
x_dot_si_sim            = zeros([2, 0]); 
x_dot_uni_sim           = zeros([2, 0]); 
x_dot_dd_sim            = zeros([2, 0]); 
poses                   = zeros([3, 0]);
sub_sample_rate         = 5;
%% Extract Trajectory Data
for i = 1:numel(files)
    if(strcmp(files(i).name(1:11), 'disturbance'))
        load(files(i).name);
        % Instantiate Trajectory Processor
        trajProcessor       = TrajectoryProcessor(pose_data, input_data, time, 'dt', dt);  
        % Obtain x_dot_si
        x_dot_si_           = trajProcessor.get_si_vels('sub_sample', sub_sample_rate, 'is_sim', 0);        % n x N x floor((iters-1)/ss)
        x_dot_si_           = trajProcessor.flatten_third_dim(x_dot_si_);                                   
        x_dot_si            = cat(2, x_dot_si, x_dot_si_);                                                  
        % Obtain x_dot_si_sim
        x_dot_si_sim_       = trajProcessor.get_si_vels('sub_sample', sub_sample_rate, 'is_sim', 1);        
        x_dot_si_sim_       = trajProcessor.flatten_third_dim(x_dot_si_sim_);                               
        x_dot_si_sim        = cat(2, x_dot_si_sim, x_dot_si_sim_);    
        % Obtain x_dot_uni_sim
        x_dot_uni_sim_      = trajProcessor.get_uni_vels('sub_sample', sub_sample_rate, 'is_sim', 1);        % n x N * floor((iters-1)/ss)
        x_dot_uni_sim_      = trajProcessor.flatten_third_dim(x_dot_uni_sim_);
        x_dot_uni_sim       = cat(2, x_dot_uni_sim, x_dot_uni_sim_);                                       
         % Obtain x_uni_uni
        x_dot_uni_          = trajProcessor.get_uni_vels('sub_sample', sub_sample_rate, 'is_sim', 0);        % n x N * floor((iters-1)/ss)                               
        x_dot_uni_          = trajProcessor.flatten_third_dim(x_dot_uni_);
        x_dot_uni           = cat(2, x_dot_uni, x_dot_uni_);      
        % Obtain x_dot_dd_sim
        x_dot_dd_sim_       = trajProcessor.get_dd_vels('sub_sample', sub_sample_rate, 'is_sim', 1);        % n x N * floor((iters-1)/ss)
        x_dot_dd_sim_       = trajProcessor.flatten_third_dim(x_dot_dd_sim_);
        x_dot_dd_sim        = cat(2, x_dot_dd_sim, x_dot_dd_sim_);                                       
         % Obtain x_dd_uni
        x_dot_dd_           = trajProcessor.get_dd_vels('sub_sample', sub_sample_rate, 'is_sim', 0);        % n x N * floor((iters-1)/ss)                               
        x_dot_dd_           = trajProcessor.flatten_third_dim(x_dot_dd_);
        x_dot_dd            = cat(2, x_dot_dd, x_dot_dd_);                                            
        % Obtain Poses 
        poses_              = trajProcessor.x(:,:,1:sub_sample_rate:end);
        poses_              = poses_(:,:,1:end-1);
        poses_              = trajProcessor.flatten_third_dim(poses_); % n x (N*iters-1)
        poses               = cat(2, poses, poses_);      % n x (N*iters-1)
    end
end
%% Fit Gaussian Process Model
% Setup Data 
labels                  = transpose(x_dot_si ./ x_dot_si_sim);                              % Labels are D(x) n x N * floor((iters-1)/ss)
X                       = transpose(poses);                                                 % size(X) = N * (iter-1)  x n  
bad_lbls                = any(abs(x_dot_si) < 1e-3, 1) | any(abs(x_dot_si_sim) < 1e-3, 1);
labels(bad_lbls, :)     = [];
X(bad_lbls, :)          = [];
x_dot_si(:,bad_lbls)    = [];
x_dot_si_sim(:,bad_lbls)= [];
inds                    = randperm(size(X, 1));                                             % Inds to Shuffle Data
X                       = X(inds, :);
x_dot_si                = x_dot_si(:, inds);
x_dot_si_sim            = x_dot_si_sim(:, inds);

for i = 1:size(labels, 2)
    y{i}                    = labels(inds, i);
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
% Plot Test Data
figure(1)
scatter(y{1}(split+1:end), y{2}(split+1:end))
hold on
scatter(ypred_test{1}, ypred_test{2})
title('Test Data', 'Interpreter', 'latex'); xlabel('$D_1$','Interpreter', 'latex'); ylabel('$D_2$', 'Interpreter', 'latex'); grid on;
ax = gca;
ax.TitleFontSizeMultiplier = 3;
% Plot Train Data
figure(2)
scatter(y{1}(1:split), y{2}(1:split))
title('Train Data', 'Interpreter', 'latex'); xlabel('$D_1$','Interpreter', 'latex'); ylabel('$D_2$', 'Interpreter', 'latex'); grid on;
ax = gca; ax.TitleFontSizeMultiplier = 3;
hold on
scatter(ypred_train{1}, ypred_train{2})
% Plot D(x) * u_si (same as x_dot_si for unicycle)
% Test Data 
figure(3)
scatter(x_dot_si(1,split+1:end), x_dot_si(2,split+1:end))
hold on
scatter(ypred_test{1}'.* x_dot_si_sim(1,split+1:end), ypred_test{2}'.* x_dot_si_sim(2,split+1:end))
title('Test Data', 'Interpreter', 'latex'); xlabel('$D_1$','Interpreter', 'latex'); ylabel('$D_2$', 'Interpreter', 'latex'); grid on;
ax = gca;
ax.TitleFontSizeMultiplier = 3;
% Train Data
absolute_truth = "I have a small penis"
rules_to_live_by = "give pablo a hug every day";