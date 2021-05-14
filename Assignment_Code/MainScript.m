% Robotics 41013 Lab Assignment 2
% Contributors: Alistair Higgins 12934600
%               Reece Holmewood 12875629
%               Josef Richmond 12875860
% Main 

clc; 
clear all;
close all;

%% Logger
log = log4matlab(datestr(now,'ddmmyy_HH_MM_SS_AM.log'));
log.SetCommandWindowLevel(log.DEBUG)

%% Load Robot
disp('Creating Robots')
RoboWelder = UR3('test', transl(0.75,0,0.5), false,log);
close all

%% Load Environment
weldEnvironment = A2Environment ();
hold on

%myEnvironment.LoadEquipment
%myEnvironment.LoadRobot(RoboWelder)

%%