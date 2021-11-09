## Robust Barrier Functions with Gaussian Processes

### Setting Up MATLAB Nodes
1. To run, you need two MATLAB nodes opened.
2. On each of the two nodes, clone this repo:
    `git clone https://github.com/yemam3/robustCBFs_with_GPs.git`
3. One node will run the experiment (robotarium node)
4. Other node will handle fitting the GPs (gp node)

### Inter-Node Communication (both nodes)

#### MQTT
1. Clone MQTT MATLAB interface:
    `git clone https://github.com/gnotomista/mqtt_matlab_interface.git`
2. The readme will contain information on how to setup mqtt using a broker (e.g. emqtt, mosquitto)
3. Set the `COMM_MODE` to `MQTT` in the `init.m` file
4. Specify the IP and PORT to be used for the communication (e.g. `localhost`, `1883`)
5. Make sure to add `mqtt_matlab_interface` to the path (edit init file)

#### File Sharing (not recommended)
1. If both nodes can share files (e.g. on same computer), we can just communicate using writing/reading files
2. Set the `COMM_MODE` to `FileSharing` in the `init.m` file

### Experiment Setting

#### Running on the Robotarium
1. Set `IS_SIM` to `0` in the `init.m` file

#### Running a Simulation         
1. Clone the Robotarium MATLAB simulator:
    `git clone https://github.com/robotarium/robotarium-matlab-simulator.git`
2. Set `IS_SIM` to `1` in the `init.m` file (this will add fake noise to the data)

### To Run Experiment
1. Run `main_robotarium.m` on the robotarium node (runs robots and collects data)
2. Run `main_gpr.m` on the gp node (this node receives the data, runs fitrgp and returns models)
