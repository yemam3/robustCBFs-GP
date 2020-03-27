%% Set up Robotarium object
% Before starting the algorithm, we need to initialize the Robotarium
% object so that we can communicate with the agents

%Initialize Robots FOR MANUAL USE ONLY!
N = 1;

x_init = [0.3;0;pi/2];

% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium('NumberOfRobots', N, 'ShowFigure', 'true','InitialConditions', x_init);

% This is a totally arbitrary number
iterations = 5000; % ~60 sec

%% Retrieve tools for single-integrator -> unicycle mapping


position_controller = create_si_position_controller('XVelocityGain', 1, 'YVelocityGain', 1);
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.75, 'AngularVelocityLimit', pi);
uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary('Disturbance',0.5);
parking_controller = create_waypoint_controller();


%% Begin the experiment

t = datetime('now');
dateString = datestr(t,'HH:MM:SS.FFF');

pose_data = zeros(3,N,iterations);
input_data = zeros(2,N,iterations);
time = zeros(1,iterations);

tic;


%Iterate for the previously specified number of iterations
for t = 1:iterations    
    %Open Loop Garbage
    dxu = repmat([0.15;0.3],1,N);
    %% Send velocities to agents    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    x = r.get_poses();
    pose_data(:,:,t) = x;
    input_data(:,:,t) =  dxu;
    time(t) = toc(tic);
    
    r.step();
end

filename = strcat('disturbanceCircleData',dateString,'.mat');
save(filename, 'pose_data', 'input_data', 'time')

