function [qMatrix,velocityMatrix,timeMatrix] = execRMRC(currentState,endPos,duration,deltaT,epsilon)
% "execRMRC" is used to calculate the relevant RMRC data to then develop
% the correct trajectory movement of the UR3.Key values are outputted
%--------------------------------------------------------------------------
% PLEASE NOTE: This code was adapted from the Lab9 Tutorial for 6DOF RMRC
%--------------------------------------------------------------------------
% >> duration = time taken for the operation
% >> deltaT = discrete time step
% >> epsilon = threshold value for Manip./DLS
% >> endPoint = goal [X Y Z] values for the movement

%% Function 1 Cut Off - INITIAL SETUP
    % UR3 params initialise
    UR3 = A2_UR3(false);
    
    % Control params.
    steps = duration/deltaT;                                                % No. of steps for simulation
        
    % Intialise Trajectory Arrays
    m = zeros(steps,1);                                                     % Array for Measure of Manipulability
    qMatrix = zeros(steps,6);                                               % Array for joint anglesR
    qdot = zeros(steps,6);                                                  % Array for joint velocities
    theta = zeros(3,steps);                                                 % Array for roll-pitch-yaw angles
    x = zeros(3,steps);                                                     % Array for x-y-z trajectory
    s = lspb(0,1,steps);                                                    % Trapezoidal trajectory scalar
    
    % Establish start and end points for execution
    currPos = UR3.model.fkine(currentState);                                % Get current position of ee
    currPos_X = currPos(1,4);                                               % X position
    currPos_Y = currPos(2,4);                                               % Y Position
    currPos_Z = currPos(3,4);                                               % Z Position
 
    % Set movement positions
    T = [eye(3) [currPos_X currPos_Y currPos_Z]'; zeros(1,3) 1];            % Create transformation of first point and angle
    q0 = currentState;                                                      % Initial guess for joint angles
    qMatrix(1,:) = UR3.model.ikcon(T,q0);                                   % Solve joint angles to achieve first waypoint
    
    for i=1:steps
        x(1,i) =  currPos_X*(1-s(i)) + s(i)*endPos(1,1);                    % Points in x
        x(2,i) =  currPos_Y*(1-s(i)) + s(i)*endPos(1,2);                    % Points in y
        x(3,i) =  currPos_Z*(1-s(i)) + s(i)*endPos(1,3);                    % Points in z
        theta(1,i) = 0;                                                     % Roll angle 
        theta(2,i) = 0;                                                     % Pitch angle
        theta(3,i) = 0;                                                     % Yaw angle
    end
    
  %% Function 2 Cut Off - TRACK TRAJECTORY
  
    for i = 1:steps-1
        T = UR3.model.fkine(qMatrix(i,:));                                  % Get forward transformation at current joint state
        deltaX = x(:,i+1) - T(1:3,4);                                       % Get position error from next waypoint
        Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                 % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3,1:3);                                                    % Current end-effector rotation matrix
        Rdot = (1/deltaT)*(Rd - Ra);                                        % Calculate rotation matrix error
        S = Rdot*Ra';                                                       % Skew symmetric!
    
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];                          % Check the structure of Skew Symmetric matrix!!
    
        deltaTheta = tr2rpy(Rd*Ra');                                        % Convert rotation matrix to RPY angles
        xdot = [linear_velocity;angular_velocity];                          % Calculate end-effector velocity to reach next waypoint.
        J = UR3.model.jacob0(qMatrix(i,:));                                 % Get Jacobian at current joint state
    
        if m(i) < epsilon                                                   % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
        else
            lambda = 0;
        end
    
        invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
        qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    
        for j = 1:6                                                         % Loop through joints 1 to 6
            if qMatrix(i,j) + deltaT*qdot(i,j) < UR3.model.qlim(j,1)        % If next joint angle is lower than joint limit...
                qdot(i,j) = 0; % Stop the motor
            elseif qMatrix(i,j) + deltaT*qdot(i,j) > UR3.model.qlim(j,2)    % If next joint angle is greater than joint limit ...
                qdot(i,j) = 0; % Stop the motor
            end
        end
        
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                   % Update next joint state based on joint velocities
        %positionError(:,i) = x(:,i+1) - T(1:3,4);                          % For plotting
        %angleError(:,i) = deltaTheta;                                      % For plotting
    end
    
    velocityMatrix = qdot(:,:);
    
    timeMatrix = 1; %set this to increment steps
end

