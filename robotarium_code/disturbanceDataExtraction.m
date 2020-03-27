files = dir('robustBarriers_disturbanceModelling/*.mat');
set(groot,'defaultLineLineWidth',4.0)
% Transformation Matrix from Differential Drive to Unicycle
r = 0.016;
l_b = 0.105;
G = [r/2, r/2; -r/l_b, r/l_b];
psi = 5;
err = [];

% Circle Data
for i = 1:numel(files)
    if(strcmp(files(i).name(1:17), 'disturbanceCircle'))
        load(files(i).name);
        trajProcessor = TrajectoryProcessor(pose_data, input_data, time, 'dt', dt);

        iters       = length(time);
    
        u_sim = trajProcessor.get_uni_vels_sim();
        u_act = trajProcessor.get_uni_vels();
        
        % Here we obtain the input to the wheels
        u_wheels_sim = trajProcessor.get_wheel_vels_sim();  % 2 x N x iters-1
        u_wheels_act = trajProcessor.get_wheel_vels();      % 2 x N x iters-1
        
        temp_err = u_wheels_act - u_wheels_sim;
        err = [err, temp_err];    
    end
end

scatter(err(1,:), err(2,:), 'filled', 'SizeData', 60);
xlim([-10, 10]);
hold on;
% Rectangular Box
plot(psi*ones(100,1), linspace(-psi,psi), 'black');
plot(-psi*ones(100,1), linspace(-psi,psi), 'black');
plot(linspace(-psi,psi), -psi*ones(100,1), 'black');
plot(linspace(-psi,psi), psi*ones(100,1), 'black');
grid on
xlabel({'$\psi{_r}$'},'Interpreter','latex', 'FontSize', 2)
ylabel({'$\psi{_l}$'},'Interpreter','latex', 'FontSize', 2)
set(gca,'FontSize',30);
sum((sum(abs(err) > psi, 1))>0)/length(err(1,:))*100;