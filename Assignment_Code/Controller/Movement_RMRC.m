%% ASSIGNMENT #2 - Real Robot Movement (RMRC)
%  Contributors: Alistair Higgins 12934600
%                Reece Holmewood  12875629
%                Josef Richmond   12875860

%% ROS SETUP
%Initialise ROS
rosinit('192.168.0.253');
%%
% Initialise the Subscriber for Real Joint States
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
% Wait for subscriber to initialise
pause(2);   
% Determine current UR3 joint states
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
% Due to default orientation being 321456, change so in correct order
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
% Set joint names
jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',...
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

%% UR3 SETUP
% Initialise UR3 class for DH parameter purposes
UR3 = A2_UR3(false);

%% TRAJECTORY SETUP
% Set control parameters
t = 10;                                                                      % Total Movement time (s)
deltaT = 0.05;                                                              % Control frequency
steps = t/deltaT;                                                           % No. of steps for simulation
delta = 2*pi/steps;                                                         % Small angle change
epsilon = 0.1;                                                              % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);                                              % Weighting matrix for the velocity vector

% Initialise the trajectory arrays
m = zeros(steps,1);                                                         % Array for Measure of Manipulability
qMatrix = zeros(steps,6);                                                   % Array for joint anglesR
qdot = zeros(steps,6);                                                      % Array for joint velocities
theta = zeros(3,steps);                                                     % Array for roll-pitch-yaw angles
x = zeros(3,steps);                                                         % Array for x-y-z trajectory
positionError = zeros(3,steps);                                             % For plotting trajectory error
angleError = zeros(3,steps);                                                % For plotting trajectory error

% Determine trajectory
s = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar

currPos = UR3.model.fkine(currentJointState_123456);                        % Get current position of ee
currPos_X = currPos(1,4);                                                   % X position
currPos_Y = currPos(2,4);                                                   % Y Position
currPos_Z = currPos(3,4);                                                   % Z Position
 
endPos = [-0.13 0.35 0.2];                                                 % Goal end pose [X Y Z]

% Set movement positions
T = [eye(3) [currPos_X currPos_Y currPos_Z]'; zeros(1,3) 1];                % Create transformation of first point and angle
q0 = currentJointState_123456;                                              % Initial guess for joint angles
qMatrix(1,:) = UR3.model.ikcon(T,q0);                                       % Solve joint angles to achieve first waypoint
for i=1:steps
    x(1,i) =  currPos_X*(1-s(i)) + s(i)*endPos(1,1);                        % Points in x
    x(2,i) =  currPos_Y*(1-s(i)) + s(i)*endPos(1,2);                        % Points in y
    x(3,i) =  currPos_Z*(1-s(i)) + s(i)*endPos(1,3);                        % Points in z
    theta(1,i) = 0;                                                         % Roll angle 
    theta(2,i) = 0;                                                         % Pitch angle
    theta(3,i) = 0;                                                         % Yaw angle
end

%% TRACK TRAJECTORY
for i = 1:steps-1
    T = UR3.model.fkine(qMatrix(i,:));                                      % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = UR3.model.jacob0(qMatrix(i,:));                                     % Get Jacobian at current joint state
    
    if m(i) < epsilon                                                       % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
    
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < UR3.model.qlim(j,1)            % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > UR3.model.qlim(j,2)        % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
    
end

%% CONVERT TRAJECTORY TO INDIVIDUAL POINTS
% Set interaction topic with UR3
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

qMatrixLength = size(qMatrix,1);
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
%goal.GoalTimeTolerance = rosduration(0.05);

for i = 1:qMatrixLength
    %tMatrix{i} = ;
    % Set the start of next motion to current state
    jointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    jointSend.Positions = qMatrix(i,:);
    %jointSend.Velocities = qdot(i,:);
    jointSend.TimeFromStart = rosduration(deltaT*i);  
        
    goal.Trajectory.Points = [goal.Trajectory.Points; jointSend];
    %goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp;
    
end
%%
%goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(5);
%%
sendGoal(client,goal);
%% SEND TRAJECTORY TO REAL UR3
points{1} = currentJointState_321456;
ctr = 1;
bufferSeconds = 0.2;

%%
while(ctr < qMatrixLength+1)
    
    % Determine current state
    currJointState = (jointStateSubscriber.LatestMessage.Position)';
    currJointState = [currJointState(3:-1:1),currJointState(4:6)];
    
    pointCheck = abs(currJointState-points{ctr});
    pcMax = max(abs(pointCheck));
    
    if pcMax < 0.0005
        
        ctr = ctr+1;
        % Set the start of next motion to current state
        startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        startJointSend.Positions = currJointState;
        startJointSend.TimeFromStart = rosduration(0.05);  
      
        endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');  
        endJointSend.Positions = points{ctr};
        endJointSend.TimeFromStart = rosduration(5);
    
        goal.Trajectory.Points = [startJointSend; endJointSend];
        
        goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp;
        sendGoal(client,goal);
        
    end
end


