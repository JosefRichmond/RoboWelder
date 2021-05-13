%% Load Robot
% <<<<<<< HEAD
close all
figure()
robot = UR5;

% Add obstacles

% Adding floor
centerpnt = [0,0,-1.01];
side = 2;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,[0,0,1],plotOptions);

planningScene = {};
planningScene{1}.vertex = vertex;
planningScene{1}.faces = faces;
planningScene{1}.faceNormals = faceNormals; 

% Adding a block in the way of the trajectory
% centerpnt = [0.38,0.1,0.05];
% side = 0.1;
% [v1,v2,v3] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,[1,0,0],plotOptions);

% planningScene{2}.vertex = v1;
% planningScene{2}.faces = v2;
% planningScene{2}.faceNormals = v3;
% 
% % 
centerpnt = [0,0.94,0.5];
side = 1;
[v1,v2,v3] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,[1,0,0],plotOptions);

planningScene{2}.vertex = v1;
planningScene{2}.faces = v2;
planningScene{2}.faceNormals = v3;
% 

%%
myController = Controller(true, robot, planningScene);

myController.makeTrajectory(a(1:20:length(a),:));
% myController.makeTrajectory([a(1,:); a(end,:)]);
myController.startMovement()
myController.humanSafe = true;
%% Control Options

% Velocity and Control Frequency

velocity = 0.1; % Desired velocity
deltaT = 1; % Control rate
x = double.empty(0,3); 
W = diag([1 1 1 0.7 0.7 0.7]);    % Weighting matrix for the velocity vector

epsilon = 0.1;% Solve joint angles to achieve first waypoint

%%  Create Goal Points
P0 = init;  %[0.537,-0.666,0.120];
P1 = [-0.3 -0.1 0.3];

%% Get trial trajectory
[p, thet] = linearTraj(P0, P1, 1, deltaT);
% p2 = p(:, 2:end);
p2 = fliplr(p);
p2 = p2(:,2:end);

pS = [p, p2]
%% 
myController.makeTrajectory(pS');
%%
myController.startMovement();
%%
Controller.humanSafe = true;
%%
Core = WelderCore(true);
 
myCore.initialiseRobot();

myCore.initialiseEnvironment()

myCore.initialiseSensor(true)

myCore.initialiseController()

myCore.getTargets()

myCore.planTrajectory()
%%
myCore.runTrajectory()
%%
qMatrix = RMRCTraj(p560,p, thet,  deltaT, epsilon, W, zeros(1,6));
%% Check if collision, if so implement collision avoidance
[collision,qInd] = IsCollision(robot.model, qMatrix,planningScene,false);
if collision
    while( collision == true)    
        qRand = qMatrix(qInd,:) +(rand(1,6)*0.5);
        while IsCollision(robot.model,qRand,planningScene,false)
            qRand = (2 * rand(1,6) - 1) * pi;
        end
        
        Pt = robot.model.fkine(qRand);
        Pt = Pt(1:3,4)';
        [p1, thet1] = linearTraj(P0, Pt, 1, deltaT);
        qMatrix1 = RMRCTraj(p560,p1, thet1,  deltaT, epsilon, W,zeros(1,6));
        [collision1,qInd] = IsCollision(robot.model, qMatrix1,planningScene,true);
        
        [p2, thet2] = linearTraj(Pt, P1, 1, deltaT);
        qMatrix2 = RMRCTraj(p560,p2, thet2,  deltaT, epsilon, W, qMatrix1(end,:));
        [collision2,qInd] = IsCollision(robot.model, qMatrix2,planningScene,true);
        
        if collision1 || collision2
            collision = true
        else
            collision = false
        end 
        
        if ~collision
            plot3(p(1,:),p(2,:),p(3,:),'k.','LineWidth',1)
            hold on
            p = [p1,p2];
            qMatrix = [qMatrix1 ; qMatrix2];
            
            plot3(p1(1,:),p1(2,:),p1(3,:),'r','LineWidth',1)
            plot3(p2(1,:),p2(2,:),p2(3,:),'b','LineWidth',1)

        end 
    end 
end 

%% SETUP A TIMER TO PLOT THE STEPS OF THE TRAJECTORY

t = timer('TimerFcn',{@plotSim, p560, qMatrix},'Period',deltaT, 'ExecutionMode', 'fixedDelay');
global ct;
ct = 1;
global safe;
safe = true;

%%
function plotSim(~,~, p560, qMatrix)
global ct; 
global safe;

if safe
ct = ct + 1

if mod(ct,2) == 0
p560.animate(qMatrix(ct,:));
end 
end 
end 

%%

%% TO RUN TIMER, TYPE t.start() in command window
%% TO STOP , TYPE t.stop()
%% If you type safe = false in the command line whilst it's running, the robot should stop
%% Type safe = true to start it again

%%

function [points, theta] = linearTraj(P0, PF, velocity, deltaT)

    dist = norm(PF - P0); % Distance from A to B
    t = dist/velocity ; % Time from A to B given v and d
    steps = ceil(t/deltaT); % Number of steps needed at given control rate
    
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
    theta(2,:) = -pi/2;
%     theta = theta';

end 

function qMatrix =  RMRCTraj(p560, x, theta, deltaT, epsilon, W, q0)
steps = length(x(1,:));
T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
% q0 = zeros(1,6);                                                            % Initial guess for joint angles
qMatrix(1,:) = p560.ikcon(T,q0);  

for i = 1:steps-2
    T = p560.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
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

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

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

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function [result,qCol] = IsCollision(robot,qMatrix,planningScene,returnOnceFound)
% if nargin < 6
%     returnOnceFound = true;
% end
result = false;
qCol = 1;
for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
    for ctr = 1:length(planningScene)
        vertex = planningScene{ctr}.vertex;
        faces = planningScene{ctr}.faces;
        faceNormals = planningScene{ctr}.faceNormals;

        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%                     plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%                     hold on 
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
%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

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

%% Load Robot


%% Generate Trajectory w/ Collision Avoidance


%% Generate Velocity Profile


