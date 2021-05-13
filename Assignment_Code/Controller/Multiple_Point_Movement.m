%% SCRIPT MOVING THE REAL UR3 
% >> THE UR3 WILL MOVE THROUGH MULTIPLE POINTS WITH THE SPEED PROPORTIONAL
% >> TO THE DISTANCE BETWEEN POINTS AND THE MOVEMENT 

%% ROS SETUP
%Initialise ROS
rosinit('192.168.0.253');

%% Initialise the Subscriber
% Initialise the Subscriber for Real Joint States
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

%% Setup initial parameters for robot connection

% Set interaction topic with UR3
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
 
%currJointState = (jointStateSubscriber.LatestMessage.Position)';
%currJointState = [currJointState(3:-1:1),currJointState(4:6)];
    
jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',...
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 20; % This is how many seconds the movement will take

currJointState = (jointStateSubscriber.LatestMessage.Position)';
currJointState = [currJointState(3:-1:1),currJointState(4:6)];

%% Set up positions required for movement 
UR3 = A2_UR3(false);

%   point = [];
%   stepAngle = 45; %in degrees
%   radius = 0.3;
%   
%   for i = 1:stepAngle:360
%       x = radius*cos(stepAngle);
%       y = radius*sin(stepAngle);
%       z = 0.35;
%       point = [point; UR3.model.ikcon(transl([x y z])*trotx(deg2rad(180)))];
%       
%   end
  %%
   point{1} = currJointState;
   point{2} = UR3.model.ikcon(transl([-0.3 -0.1 0.3])*trotx(deg2rad(180)));   %deg2rad([-69.94 0.2 -51.91 -156.85 -130.30 348.60]);
   point{3} = UR3.model.ikcon(transl([-0.3 -0.11 0.2])*trotx(deg2rad(180)));  %deg2rad([-69.94 17.46 -65.26 -160.72 -130.26 348.65]);
   point{4} = UR3.model.ikcon(transl([-0.3 -0.12 0.3])*trotx(deg2rad(180)));  %deg2rad([-94.23 24.02 -92.18 -134.32 -108.22 0.57]);
   point{5} = UR3.model.ikcon(transl([-0.3 -0.13 0.2])*trotx(deg2rad(180)));  %deg2rad([-111.82 1.58 -80.34 -123.47 -92.05 7.45]);
   point{6} = UR3.model.ikcon(transl([-0.3 -0.1 0.3])*trotx(deg2rad(180)));   %deg2rad([-69.94 0.2 -51.91 -156.85 -130.30 348.60]);
   point{7} = UR3.model.ikcon(transl([-0.3 -0.11 0.2])*trotx(deg2rad(180)));  %deg2rad([-69.94 17.46 -65.26 -160.72 -130.26 348.65]);
   point{8} = UR3.model.ikcon(transl([-0.3 -0.12 0.3])*trotx(deg2rad(180)));  %deg2rad([-94.23 24.02 -92.18 -134.32 -108.22 0.57]);
   point{9} = UR3.model.ikcon(transl([-0.3 -0.13 0.2])*trotx(deg2rad(180)));  %deg2rad([-111.82 1.58 -80.34 -123.47 -92.05 7.45]);
   point{10} = UR3.model.ikcon(transl([-0.3 -0.1 0.3])*trotx(deg2rad(180)));   %deg2rad([-69.94 0.2 -51.91 -156.85 -130.30 348.60]);
   point{11} = UR3.model.ikcon(transl([-0.3 -0.11 0.2])*trotx(deg2rad(180)));  %deg2rad([-69.94 17.46 -65.26 -160.72 -130.26 348.65]);
   point{12} = UR3.model.ikcon(transl([-0.3 -0.12 0.3])*trotx(deg2rad(180)));  %deg2rad([-94.23 24.02 -92.18 -134.32 -108.22 0.57]);
   point{13} = UR3.model.ikcon(transl([-0.3 -0.13 0.2])*trotx(deg2rad(180)));  %deg2rad([-111.82 1.58 -80.34 -123.47 -92.05 7.45]);
   qMat = [];
 
  for i = 1:13
      qMat = [qMat;point{i}];
  end
%% MOVE FROM CURRENT STATE TO FIRST POINT
% Set the start of next motion to current state

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currJointState;
%startJointSend.Velocities = [pi/9 pi/9 pi/9 pi/9 pi/9 pi/9];
startJointSend.TimeFromStart = rosduration(0.05);  
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');  
endJointSend.Positions = point(1,:);
%endJointSend.Velocities = [pi/9 pi/9 pi/9 pi/9 pi/9 pi/9];
endJointSend.TimeFromStart = rosduration(5);
    
goal.Trajectory.Points = [startJointSend; endJointSend];
        
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(1);

sendGoal(client,goal);
%%

%numPos = size(qMat,1); %size(point);
 
qMatrixLength = size(q,1);
 
 %% Attempt at concatenating the qValues and sending at once
 
deltaT = durationSeconds/qMatrixLength;

for i = 1:qMatrixLength
    
    % Set the start of next motion to current state
    jointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    jointSend.Positions = q(i,:);
    jointSend.TimeFromStart = rosduration(deltaT*i);  
        
    goal.Trajectory.Points = [goal.Trajectory.Points; jointSend];
        
end

goal.Trajectory.Header.Stamp = rostime(0);


%%

sendGoal(client,goal);

 %% Execute movement
 Simple_Movement(qMat, client, goal, jointStateSubscriber, 20);
 
 %% FUNCTION TEST
 
 [client, goal, jointStateSubscriber,jointNames] = InitialiseReal('192.168.0.253');
 %%
 [currJointState] = getCurrentState(jointStateSubscriber);
 
 [goal] = createTransmission(20, goal, q);
 %%
 sendTransmission(client,goal);
 
 %% FUNCTIONS
function [client, goal,jointStateSubscriber, jointNames] = InitialiseReal(raspIP)
    %Initialise ROS
    rosinit(raspIP);

    % Initialise the Subscriber for Real Joint States
    jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

    % Set interaction topic with UR3
    [client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
    
    % Set joint names
    jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',...
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
    goal.Trajectory.JointNames = jointNames;
    
    % Set remaining goal paratmeters
    goal.Trajectory.Header.Seq = 1;                                         % check usefulness
    goal.Trajectory.Header.Stamp = rostime('Now','system');                 % check usefulness
    goal.GoalTimeTolerance = rosduration(0.05);                             % check usefulness

end

function[currJointState] = getCurrentState(jointStateSubscriber)
    % Pull current joint states from the joint state subscriber
    % !! Note that the original order is 3-2-1-4-5-6 !!
    currJointState = (jointStateSubscriber.LatestMessage.Position)';
    % Transfer joint order to 1-2-3-4-5-6
    currJointState = [currJointState(3:-1:1),currJointState(4:6)];
end

function[goal] = createTransmission(duration, goal, qMatrix)
    
    % Determine number of steps within the qMatrix
    qMatrixLength = size(qMatrix,1);
    % Sets the discrete time step
    deltaT = duration/qMatrixLength;

    for i = 1:qMatrixLength
    
        % Set the start of next motion to current state
        jointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        jointSend.Positions = qMatrix(i,:);
        jointSend.TimeFromStart = rosduration(deltaT*i);  
        
        goal.Trajectory.Points = [goal.Trajectory.Points; jointSend];
        
    end

    goal.Trajectory.Header.Stamp = rostime(0);
    
end

function sendTransmission(client,goal)
    % Send the generated real trajectory (in "goal")
    sendGoal(client,goal);
    clear goal
end