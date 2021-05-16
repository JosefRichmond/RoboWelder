%% BEFORE RUNNING 1ST TIME OR WHEN CHANGING MODE OF OPERATION
% IF SIMULATED BOTH SENSOR AND CONTROLLER
% Run - roslaunch usb_cam usb_cam-test.launch

% IF SIMULATED SENSOR AND NOT CONTROLLER
% Run - roslaunch usb_cam usb_cam-test.launch

% IF USING REAL SENSOR AND CONTROLLER
% Run - roslaunch freenect_launch freenect.launch depth_registration:=true
% Run - roslaunch usb_cam usb_cam-test.launch

%% INITIALISE CORE & ROS
myCore = WelderCore(false);

%% LOAD ENVIRONMENT
myCore.initialiseEnvironment()

%% LOAD ROBOT INTO FIGURE
myCore.initialiseRobot();

%% LOAD UP, FIRST ARGUMENT MEANS SIMULATED OR NOT, LEAVE AS TRUE SO WE DON'T HAVE TO CONNECT TO REAL CAMS
myCore.initialiseSensor(false,true)

%% LOAD CONTROLLER, FIRST ARGUMENT MEANS SIMULATED OR NOT
myCore.initialiseController(false)

%% GETS THE TARGETS FROM ROSBAG
myCore.getTargets()

%% PLANS AND PLOTS TRAJECTORY
myCore.planTrajectory()


%% EITHER RUNS THE SIMULATED VERSION OR SENDS THE MSG TO ROS
myCore.runTrajectory()

%% CALL THIS TO SHUT DOWN ALL PROCESSES BEFORE YOU MAKE A NEW CORE 
myCore.Controller.stopMovement()
myCore.Sensor.deactivateHumanDetection()
%%
close all
clear