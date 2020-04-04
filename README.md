## Instructions For Running On The Robotarium Using MQTT

### If You Are Running A Simulation

1. If you are simulating, clone the Robotarium MATLAB simulator:
    `git clone https://github.com/robotarium/robotarium-matlab-simulator.git`
2. Clone this repo:
    `git clone https://github.com/yemam3/robustCBFs_with_GPs.git`
3. Clone MQTT MATLAB interface:
    - `git clone https://github.com/gnotomista/mqtt_matlab_interface.git`
    - The readme will contain information on how to setup mqtt using a broker (e.g. emqtt, mosquitto)
4. If you are running this on two devices make sure to repeat 2 and 3 for each
5. Make sure all repos are on MATLAB path except (TODO: may need to edit the init files)  

### To Run Experiment

1. Run `main_mqtt.m` on the robotarium node (runs robots and collects data)
2. Run `gpr_mqtt.m` on the other node (this node receives the data, runs fitrgp and returns sigmas)
