%% CODE FOR MOVEMENT OF SIMULATED ROBOT
% Robotics 41013 Lab Assignment 2
% Contributors: Alistair Higgins 12934600
%               Reece Holmewood 12875629
%               Josef Richmond 12875860

% NOTE: Adapted from lab 11 code walkthrough to be used for our UR3

%% JOYSTICK SETUP
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

%% ROBOT MOVEMENT
robot = A2_UR3(false,true);
hold on;

[self,f,v] = A2Environment();

q = deg2rad([0 -90 0 -90 0 0]);                 % Set initial robot configuration 'q'
robot.model.delay = 0.001;                      % Set smaller delay when animating

duration = 200;                                 % Set duration of the simulation (seconds)
dt = 0.4;                                       % Set time step for simulation (seconds)

n = 0;                                          % Initialise step count to zero 
tic;                                            % recording simulation start time

%pause;

while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
    % Joystick Axes description
    % All axes values range from -1.000 to +1.000
    % -------------------------------------------------------------
    % Axes[1] = Left Joystick 	(LEFT to RIGHT)
    % Axes[2] = Left Joystick   (UP to DOWN)
    % Axes[3] = Left Trigger    (NO-PRESS to PRESSED)
    % Axes[4] = Right Joystick  (LEFT to RIGHT)
    % Axes[5] = Right Joystick  (UP to DOWN)
    % Axes[6] = Right Trigger   (NO-PRESS to PRESSED)
    % Axes[7] = D-Pad           (LEFT to RIGHT)
    % -------------------------------------------------------------
    
    % YOUR CODE GOES HERE
    % SECTION 1: Turn joystick input into an end-effector velocity command
    
    % Gain variables
    linGain = 0.1;
    angGain = 0.5;
    
    % Linear Variables
    vx = linGain * axes(1);                   % left joystick left and right
    vy = linGain * axes(2);                   % left joystick up and down
    vz = linGain * (axes(6)-axes(3))/2;       % left and right trigger
    
    % Angular Variables
    wx = angGain * axes(4);                   % right joystick left and right
    wy = angGain * axes(5);                   % right joystick up and down
    wz = angGain * axes(7);                   % d-pad left and right
    
    % Combined Velocity Vector
    dx = [vx;vy;vz;wx;wy;wz]; 
    
    % Set a base cutoff to limit "drift"
    dx((dx.^2)<0.01) = 0;
    disp(dx);
    
    % 2 - use J inverse to calculate joint velocity
    J = robot.model.jacob0(q);
    
    % Incorporate the calc. of DLS
    lambda = 0.1;
    Jinv_dls = inv((J'*J)+lambda^2*eye(6))*J';
    
    dq = Jinv_dls*dx;
    %dq = inv(J)*dx;
    
    % 3 - apply joint velocity to step robot joint angles 
    q = q + (dq*dt)';
    
    % Update plot
   
    robot.model.animate(q);  
    %hold on;
    %trplot(robot.fkine(q));
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end
      
