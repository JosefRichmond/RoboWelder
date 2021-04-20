%% Housekeeping

clc
clear all
close all
%% Initialise ROS Master Node

rosinit
%% Create a sample network

exampleHelperROSCreateSampleNetwork

%% Check the subscribers to the laser message, note that the matlab node is not one of them

rostopic info /scan
%%  Subscribe to the laser message

laser = rossubscriber('/scan');
%% Now we can see that the Matlab node is subscribed to the laser message

rostopic info /scan
%% Receive some data from the laser

% Using receive just gets all the data that the subscriber has collected
scandata = receive(laser,10)

figure
plot(scandata,'MaximumRange',7)
%% Setup a function to run every time new data is received from a message (the bad way)

% This sets a subscriber to the pose message, and the function
% exampleHelperROSPoseCallback is called every time the pose message sends
% new data
a = 57;
robotpose = rossubscriber('/pose',@exampleHelperROSPoseCallback)

% The data is stored in global variables, this is dumb
global pos
global orient

pause(2)

%% Read the data stored in pos & orient by the subscriber callback
pos
orient

%% Stop listening to the pose message

clear robotpose
%% Setup a function to run every time new data is received from a message (the better way)

% Make an instance of a simple class that has two properties
% editProperty - which i've included to show that you can edit class
% properties in a callback function
% receiveProperty - to show that you can access class properties in a
% callback function
testClass = SimpleHandleClass(10);
robotpose = rossubscriber('/pose',{@goodCallBack,testClass});

%% Read the data stored in pos & orient by the subscriber callback

for i = 1:10
    pause(0.1)
    display(num2str(testClass.editProperty))
end 

%% Stop the subcription to pose message

clear robotpose

%% Shutdown ROS nodes

rosshutdown