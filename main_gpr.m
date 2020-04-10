%% Disturbance Estimation for GritsBot-Xs on the Guassian Process Node (Main Script)
% Yousef Emam
% 21/03/2020
% Main Robotarium Script.

% Initialization File
init; 
% Instantiate Disturb Estimator Object 
disturb_estimator = DisturbanceEstimator(N,n,m,IS_SIM,COMM_MODE,IP,PORT);

% Run This Loop Forever (Receive Data, Fit GP Models, Send Models)
counter = 1;
while true 
    disturb_estimator = disturb_estimator.main();
    % Save Everything every steps 10 roughly
    if mod(counter,100) == 1
        save(['saved_data/gpr_mqtt_workspace_', date_string,'.mat']);
    end
    counter = counter + 1;
    pause(0.1)
end




