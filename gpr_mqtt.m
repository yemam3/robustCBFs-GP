init; init_mqtt; date_string = datestr(datetime('now'),'HH:MM:SS.FFF');
n = 3; m = 2; N = 3;
disturb_estimator = DisturbanceEstimator_mqtt(N, n, m);
t = 1;
while true 
    disturb_estimator = disturb_estimator.main();
    if mod(t, 10) == 1
        save(['saved_data/gpr_mqtt_workspace_', date_string,'.mat']);
    end
    t = t + 1;
    pause(0.1)
end


