init; init_mqtt;
n = 3; m = 2; N = 3;
disturb_estimator = DisturbanceEstimator_mqtt(N, n, m);

while true 
    disturb_estimator = disturb_estimator.main();
    pause(0.1)
end