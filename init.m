clc
clear
close all
%clear java
%rng(10) % Fix Random Seed

% Changes Current Directory to src and recursively add children
repo_path = fileparts(mfilename('fullpath')); 
addpath(genpath(repo_path));

% Initialize Experimental Details
IS_SIM                          = 1;                            % Is this a Simulation? (if it is, we'll add some fake noise)
COMM_MODE                       = 'FileSharing';                % Inter-Node Communication Mode ('FileSharing' or 'MQTT')
CBF_SPECS.cbf_mode              = 'Multiplicative';                    % No/Additive/Multiplicative Disturbance: 'Regular', 'Additive', 'Multiplicative' 
CBF_SPECS.safety_radius         = 0.110;                        % How big is the safety radius between robots?
CBF_SPECS.nominal_radius        = 0.110;                        % How big is the circle in reality? 
CBF_SPECS.barrier_gain          = 100;
CBF_SPECS.projection_distance   = 0.015;
N                               = 4;                            % Number of Robots
n                               = 3;                            % Dimension of state x_i (don't change)
m                               = 2;                            % Dimension of control u_i  (don't change)
iterations                      = 10000/4;                      % Select the number of iterations for the experiment.
date_string = datestr(datetime('now'),'HH:MM:SS.FFF');  % For Data Saving Purposes
% Define IP & Port to be used 
IP                              = 'localhost';                  % Robotarium: '192.168.1.8' Local: 'localhost'
PORT                            = 1883;                         % Robotarium: 1884          Local: 1883
% Data Saving Path
SAVE_PATH                       = ['saved_data/robotarium_', CBF_SPECS.cbf_mode, '_', num2str(CBF_SPECS.safety_radius*100), '_', date_string, '/'];
mkdir(SAVE_PATH);
% Suppress is not serializable warning (caused when saving data)
warning('off', 'MATLAB:Java:ConvertFromOpaque');

%% If we're using MQTT Need to Set some parameters
if strcmp(COMM_MODE, 'MQTT')
    if IS_SIM
        % PERSONAL DEVICE
        addpath(genpath('/Users/yousefemam/Google Drive/mqtt_matlab_interface/'))
        javaaddpath('/Users/yousefemam/Google Drive/mqtt_matlab_interface/jars/org.eclipse.paho.client.mqttv3-1.2.2.jar')
        javaaddpath('/Users/yousefemam/Google Drive/mqtt_matlab_interface/jars/iMqttClient.jar');
    else
        % FOR ROBOTARIUM
        addpath(genpath('/home/user_code/mqtt_matlab_interface/'))
        javaaddpath('/home/user_code/mqtt_matlab_interface/jars/org.eclipse.paho.client.mqttv3-1.2.2.jar')
        javaaddpath('/home/user_code/mqtt_matlab_interface/jars/iMqttClient.jar');
    end
end