%% FIRST INITIAL ROBOT MOVEMENT
%% Initialise ROS
rosinit('192.168.0.253');

%% Initialise the Subscriber
% Initialise the Subscriber for Real Joint States
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

%% Check Joint States
pause(2); %Wait 2 seconds

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)]

jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',...
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

%% GAVIN's EXAMPLE

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');

goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 5; % This is how many seconds the movement will take

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
      
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

nextJointState_123456 = [0,0,0,0,0,0];

endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% RUN

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);


%% Run through multiple different locations

% Set interaction topic with UR3
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
 
currJointState = (jointStateSubscriber.LatestMessage.Position)';
currJointState = [currJointState(3:-1:1),currJointState(4:6)];
    
jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',...
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);

bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 2; % This is how many seconds the movement will take

currJointState = (jointStateSubscriber.LatestMessage.Position)';
currJointState = [currJointState(3:-1:1),currJointState(4:6)];

point = [];

point{1} = currJointState;
point{2} = deg2rad([-69.94 0.2 -51.91 -156.85 -130.30 348.60]);
point{3} = deg2rad([-69.94 17.46 -65.26 -160.72 -130.26 348.65]);
point{4} = deg2rad([-94.23 24.02 -92.18 -134.32 -108.22 0.57]);
point{5} = deg2rad([-111.82 1.58 -80.34 -123.47 -92.05 7.45]);

numPos = size(point);
%%
ctr = 1;

while ctr < 6
    % Determine current state
    currJointState = (jointStateSubscriber.LatestMessage.Position)';
    currJointState = [currJointState(3:-1:1),currJointState(4:6)];
    
    pointCheck = abs(currJointState-point{ctr});
    pcMax = max(pointCheck);
    
    if pcMax < 0.0005
        
        ctr = ctr+1;
        % Set the start of next motion to current state
        startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        startJointSend.Positions = currJointState;
        startJointSend.TimeFromStart = rosduration(0.05);  
      
        endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');  
        endJointSend.Positions = point{ctr};
        endJointSend.TimeFromStart = rosduration(durationSeconds);
    
        goal.Trajectory.Points = [startJointSend; endJointSend];
        
        goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
        sendGoal(client,goal);
        
    end
    
end


%% VELOCITY CONTROL FOR REAL UR3
%% Initialise ROS
rosinit('192.168.0.253');
%%
UR3 = A2_UR3(false)
%% Initialise the Subscriber
% Initialise the Subscriber for Real Joint States
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');

pause(2);                                                                   % Wait for subscriber to initialise

%% Determine Initial Joint States
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'

currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)]

jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',...
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

%% Parameter Setup                         

t = 5;                                                                      % Total Movement time (s)
deltaT = 0.02;                                                              % Control frequency
steps = t/deltaT;                                                           % No. of steps for simulation
delta = 2*pi/steps;                                                         % Small angle change
epsilon = 0.1;                                                              % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);                                              % Weighting matrix for the velocity vector

%% Initialise Arrays (empty)
m = zeros(steps,1);                                                         % Array for Measure of Manipulability

qMatrix = zeros(steps,6);                                                   % Array for joint anglesR

qdot = zeros(steps,6);                                                      % Array for joint velocities

theta = zeros(3,steps);                                                     % Array for roll-pitch-yaw angles

x = zeros(3,steps);                                                         % Array for x-y-z trajectory

positionError = zeros(3,steps);                                             % For plotting trajectory error
angleError = zeros(3,steps);                                                % For plotting trajectory error

%% Determine Trajectory for 2 points

s = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar

currPos = UR3.model.fkine(currentJointState_123456);                       % Get current position of ee
currPos_X = currPos(1,4);                                                   % X position
currPos_Y = currPos(2,4);                                                   % Y Position
currPos_Z = currPos(3,4);                                                   % Z Position
 
endPos = [0.35 0.1 0.1865];                                                     % Goal end pose [X Y Z]

for i=1:steps
    x(1,i) =  currPos_X*(1-s(i)) + s(i)*endPos(1,1);                        % Points in x
    x(2,i) =  currPos_Y*(1-s(i)) + s(i)*endPos(1,2);                        % Points in y
    x(3,i) =  currPos_Z*(1-s(i)) + s(i)*endPos(1,3);                        % Points in z
    theta(1,i) = 0;                                                         % Roll angle 
    theta(2,i) = 0;                                                         % Pitch angle
    theta(3,i) = 0;                                                         % Yaw angle
end
%%
T = [eye(3) [currPos_X currPos_Y currPos_Z]'; zeros(1,3) 1];                % Create transformation of first point and angle
q0 = currentJointState_123456;                                              % Initial guess for joint angles
qMatrix(1,:) = UR3.model.ikcon(T,q0);                                       % Solve joint angles to achieve first waypoint

%% Track the trajectory with RMRC
for i = 1:steps-1
    T = UR3.model.fkine(qMatrix(i,:));                                      % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = UR3.model.jacob0(qMatrix(i,:));                                          % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                                       % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < UR3.model.qlim(j,1)                 % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > UR3.model.qlim(j,2)             % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end






