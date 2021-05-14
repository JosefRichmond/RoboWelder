classdef Controller < handle
%UNTITLED Summary of this class goes here
%   Detailed explanation goes here

properties
    
    % Options
    sim % true or false for simulated or not
    controlOptions %Struct containing control options  (velocity, epsilon, weight matrix etc)
    planningScene %Struct containing the faces, facenormals and vertices of all the objects in the environment.
    
    % Safety
    humanSafe = true
    
    % SerialLink
    robot
    
    % Sensor Inputs
    humanSub %Links to publisher of human data

    % Trajectory Planning
    plannedQ % Designed Trajectory Q Matrix
    plannedPoints % Designed Trajectory Points
    goalQ % Requested Q Matrix 
    goalPoints % Requested Points
    
    % Simulation Movements
    trajState = 1 % Trajectory Step
    moveTimer % timer for plotting
    
    % Real robot IP address
    raspIP = '192.168.0.253';
    client
    goal
    jointStateSubscriber
    jointNames
    
    
end 

methods
        
%% CONSTRUCTOR
function obj = Controller(Sim, robot,planningScene)
    % Constructor for controller, doesn't do anything other than
    % establishes if simulation or note
    obj.sim = Sim;
    obj.robot = robot;
    obj.updatePlanningScene(planningScene);
    
    if ~Sim
            [obj.client, obj.goal,obj.jointStateSubscriber, obj.jointNames] = obj.InitialiseReal();
    end
    
end
    
%% UPDATE PLANNING SCENE
function updatePlanningScene(obj, planningScene)
    % Function that adds the passed planning scene data to the
    % current planning scene

    obj.planningScene = planningScene;
    obj.humanSafe = false; 

end 

%% MAKE TRAJECTORY GIVEN WAYPOINTS
function [qs, xs] = makeTrajectory(obj, points)
%MAKETRAJECTORY Creates trajectory to send to UR3 or simulator
%   Detailed explanation goes here
    
% Control variables
robot = obj.robot.model;
planningScene = obj.planningScene;
deltaT = 0.01;
epsilon = 0.1;
W = diag([1 1 1 0.7 0 0]); 
q0 = zeros(1,6);
xs = double.empty(3,0); 
qs = double.empty(0,6);
xg = double.empty(3,0);
qg = double.empty(0,6);
% TF = ones(4,4);

TF = transl(0,0,0);          % Create transformation of first point and angle

for ctr = 1:length(points(:,1))-1
    P0 = points(ctr,:);
    P1 = points(ctr+1,:);
    
    [p, thet] = obj.linearTraj(P0, P1, 1, deltaT);
    qMatrix = obj.RMRCTraj(robot,p, thet,  deltaT, epsilon, W, q0);
    [collision,qInd] = obj.IsCollision(robot, qMatrix,planningScene,false); %#ok<*PROPLC>
    
    xg = [xg, p];
    qg = [qg ; qMatrix];
    
    if collision
        while( collision == true)    
            qRand = qMatrix(qInd,:) +(rand(1,6)*0.5);
            while obj.IsCollision(robot,qRand,planningScene,false)
                qRand = (2 * rand(1,6) - 1) * pi;
            end

            Pt0 = robot.fkine(qRand);
            Pt = Pt0(1:3,4)';
            [p1, thet1] = obj.linearTraj(P0, Pt, 1, deltaT);
            qMatrix1 = obj.RMRCTraj(robot,p1, thet1,  deltaT, epsilon, W,qg(end,:));
            [collision1,qInd] = obj.IsCollision(robot, qMatrix1,planningScene,true);
                
            q0 = qMatrix1(end,:);
            
            [p2, thet2] = obj.linearTraj(Pt, P1, 0.5, deltaT);
            qMatrix2 = obj.RMRCTraj(robot,p2, thet2,  deltaT, epsilon, W, q0);qs
            [collision2,qInd] = obj.IsCollision(robot, qMatrix2,planningScene,true);
            
            q0 = qMatrix2(end,:);

            if collision1 || collision2
                collision = true
            else
                collision = false
            end 

            if ~collision
                
%                 plot3(p(1,:),p(2,:),p(3,:),'k.','LineWidth',1)
                hold on
                p = [p1,p2];
                qMatrix = [qMatrix1 ; qMatrix2];
%                 plot3(p1(1,:),p1(2,:),p1(3,:),'r','LineWidth',1)
%                 plot3(p2(1,:),p2(2,:),p2(3,:),'b','LineWidth',1)
               
                qs = [qs; qMatrix];
                xs = [xs, p];

            end 
        end
    else
        qs = [qs; qMatrix];
        xs = [xs, p];
%         plot3(xs(1,:),xs(2,:),xs(3,:),'k.','LineWidth',1)
        hold on
    end
    
    TF = robot.fkine(qMatrix(end,:));
end 

obj.plannedQ = qs;
obj.goalQ = qg;
obj.plannedPoints = xs;
obj.goalPoints = xg; 

plot3(xs(1,:),xs(2,:),xs(3,:),'g.','LineWidth',1)
plot3(xg(1,:),xg(2,:),xg(3,:),'r','LineWidth',1)

end
   
%% MAKE LINEAR TRAJECTORY BETWEEN 2 POINTS
function [points, theta] = linearTraj(obj, P0, PF, velocity, deltaT)

    dist = norm(PF - P0); % Distance from A to B
    t = dist/velocity ; % Time from A to B given v and d
    steps = 2*ceil(t/deltaT); % Number of steps needed at given control rate

    temp_x = (linspace(P0(1), PF(1), steps))';
    temp_y = (linspace(P0(2), PF(2),steps))';
    temp_z = (linspace(P0(3), PF(3), steps))';

    if isempty(temp_x)
        temp_x = ones(steps,1)*PF(1);
    end 
    if isempty(temp_y)
        temp_y = ones(steps,1)*PF(2);
    end 
    if isempty(temp_z)
        temp_z = ones(steps,1)*PF(3);
    end 
    points = [temp_x, temp_y, temp_z]';

    theta = zeros(3,steps);
%     theta(3,:) = -pi/2;%-pi/2;
%     theta(2,:) = pi;%-pi/2;
    theta(1,:) = -pi/2;

end 
    
%% MAKE RMRC CONTROLLED QMATRIX GIVEN LINEAR TRAJECTORY
function qMatrix =  RMRCTraj(obj,p560, x, theta, deltaT, epsilon, W, q0)
steps = length(x(1,:));

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
% q0 = zeros(1,6);                                                            % Initial guess for joint angles
qMatrix(1,:) = p560.ikcon(T,q0);  
% TF = T;
for i = 1:steps-2
    T = p560.fkine(qMatrix(i,:));
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];
    try % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');       
    catch
        adh =1;
    end % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = p560.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

end 
               
%% CHECK Q MATRIX TRAJECTORY FOR A COLLISION
function [result,qCol] = IsCollision(obj, robot,qMatrix,planningScene,returnOnceFound)
% if nargin < 6
%     returnOnceFound = true;
% end
result = false;
qCol = 1;
for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = obj.GetLinkPoses(qMatrix(qIndex,:), robot);
    
    for ctr = 1:length(planningScene)
        vertex = planningScene{ctr}.vertex;
        faces = planningScene{ctr}.faces;
        faceNormals = planningScene{ctr}.faceNormals;

        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = obj.LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && obj.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'r*');
                    hold on 
%                     display('Intersection');
                    result = true;
                    if qCol == 0
                        qCol = qIndex; 
                    end 
                    if returnOnceFound
                        display('reslt')
                        return
                    end
                end
            end    
        end
    end 
end
end

%% BEGIN MOVEMENT
function startMovement(obj)
            % Either sends the trajectory to ROS or to the simulator
            % If it is to ROS
            %   Create message and send via ros send function. Listens to
            %   sensor publisher and soft stops the robot if there is a
            %   human detected or e-stop etc.
            % If it is to simulator
            %   Create timer that calls back to plot function at control
            %   rate and plots robot movements 
%             obj.safe = true;

    if ~obj.sim
        duration = 10; % seconds;
        
        [goal] = obj.createTransmission(duration, obj.goal);
        
        obj.sendTransmission(obj.client,goal);
        
    else 
        obj.trajState = 1;
        if isempty(obj.moveTimer)
            obj.moveTimer = timer('TimerFcn',{@obj.plotSim},'Period',0.02*10, 'ExecutionMode', 'fixedDelay');
        end 
        obj.moveTimer.start();
    end 
end 

%% MOVE SIMULATED ROBOT
function plotSim(obj, ~,~)
robot = obj.robot.model;
qMatrix = obj.plannedQ;
if obj.humanSafe
obj.trajState = obj.trajState + 1;

if mod(obj.trajState,2) == 0
robot.animate(qMatrix(obj.trajState-1:obj.trajState,:));
end 

if obj.trajState > length(obj.plannedQ(:,1))
    obj.moveTimer.stop()
    display('Trajectory Completed')
    obj.startMovement();
end 

end 
end 

%% MOVE REAL ROBOT FUNCTIONS
function [client, goal,jointStateSubscriber, jointNames] = InitialiseReal(obj)
    %Initialise ROS
    %rosinit(raspIP);

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

%% COLLECT CURRENT STATE
function[currJointState] = getCurrentState(obj)
    % Pull current joint states from the joint state subscriber
    % !! Note that the original order is 3-2-1-4-5-6 !!
    currJointState = (obj.jointStateSubscriber.LatestMessage.Position)';
    % Transfer joint order to 1-2-3-4-5-6
    currJointState = [currJointState(3:-1:1),currJointState(4:6)];
end

%% CREATE TRANSMISSION TO SEND TO ROBOT
function[goal] = createTransmission(obj,duration, goal)
    
    %Get current state and add to start of transmission
    [currJointState] = obj.getCurrentState();
    jointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    jointSend.Positions = currJointState;
    jointSend.TimeFromStart = rosduration(0.5);
    goal.Trajectory.Points = [goal.Trajectory.Points];
    
    % Set time stamp for next position
    timeStamp = 10;
        
    % Determine number of steps within the qMatrix
    qMatrixLength = size(obj.plannedQ,1);
    % Sets the discrete time step
    deltaT = duration/qMatrixLength;
    
    for i = 1:qMatrixLength

        % Set the start of next motion to current state
        jointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        jointSend.Positions = obj.plannedQ(i,:);
        jointSend.TimeFromStart = rosduration(timeStamp);  
        
        goal.Trajectory.Points = [goal.Trajectory.Points; jointSend];
        
        % Increment the time from start variable
        timeStamp = timeStamp + deltaT;
    end

    goal.Trajectory.Header.Stamp = rostime(0);
    
end

%% TRANSMIT THE ACTION TO THE ROBOT
function sendTransmission(obj,client,goal)
    % Send the generated real trajectory (in "goal")
    sendGoal(client,goal);
    clear goal
end

%% Deactivate Sensor
function stopMovement(obj)
    try
        obj.moveTimer.stop();
    end 
end 

%% CONNECT TO HUMAN DETECTION
function connectHuman(obj, Sensor)
    try 
        obj.humanSub = rossubscriber(Sensor.humanPub.TopicName, @obj.dummy);
    catch
        display("Could not connect to human detection camera")
    end
end 

%% FUNCTION THAT 
function dummy(obj,~,message)
   
    if strcmp('false', message.Data)
        obj.humanSafe = true;
    else
        obj.humanSafe = false;
    end 
    
end 

%% IsIntersectionPointInsideTriangle - GIVEN CODE, NOT ORIGINAL WORK
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(obj, intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% GetLinkPoses - GIVEN CODE, NOT ORIGINAL WORK
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses(obj, q, robot)

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% LinePlaneIntersection
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = LinePlaneIntersection(obj, planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;       %no intersection
        return
    end
end

%compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end
    
    end  
end

