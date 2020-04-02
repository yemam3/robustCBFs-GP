clc
clear
close all

% Changes Current Directory to Source and adds necessary files
ParentFolderPath = fileparts(mfilename('fullpath')); % path to icra_simulations
addpath(genpath('mqtt_mode'));
addpath(genpath('/Users/yousefemam/Google Drive/robotarium'));