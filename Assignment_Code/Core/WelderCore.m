classdef WelderCore < handle
    %WELDERCORE Controls decision making processes for the welder robot
    %   The Welder Core class contains the functionality used in the
    %   decision making processes required to control the real and
    %   simulated UR3 welding robots. The main function of the WelderCore
    %   is to initialise instances of the Sensor and Controller classes,
    %   and connect them to the correct ROS nodes depending on whether live
    %   or simulated data is used. 
    
    properties
        sim
        Sensor
        Controller
        Robot
        PlanningScene
        Axes 
        Target
    end
    
    methods
%% CONSTRUCTOR
function obj = WelderCore(sim)
    %CORE Construct an instance of this class
    %   Will initialise the ROS node. In the case that this is a
    %   simulation, the ROS master will be located on the computer, for a
    %   real demo the ros master will connect to the Raspberry Pi.
    
    obj.sim = sim;
    try
        if sim
            rosinit % ROS master on computer
        else 
            rosinit('192.168.0.253') % ROS master on Raspberry Pi
        end 
    end 

end

%% INITIALISE WELDING UR3 ROBOT
function initialiseRobot(obj)
    %INITIALISEROBOT Initalises an instance of the A2_UR3 robot class and
    %plots it
    obj.Robot = A2_UR3(false,true);
    
end 

%% INITIALISE SIMULATED ENVIRONMENT
function initialiseEnvironment(obj, loadEnvironment)
    %INITIALISEENIVRONMENT initialises the simulated environment
    %   Loads the simulated environment from .ply files. The faces,
    %   vertices and face normals of each polygon are loaded into the
    %   planning scene which is later sent to the controller. This planning
    %   scene enables the trajectory generator to detect and avoid
    %   collisions.
    
    % Load welding cell and respective safety models
    [enviro,face,vertex] = A2Environment();
    hold on
    
    % Get faceNormals data from the A2 Environment outputs
    faceNormals = zeros(size(face,1),3);
   
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end   
   
    % For each set of face,  vert and Facenorm add to cell array
    % planningSCene
    
    planningScene = {};

    planningScene{1}.vertex = vertex;
    planningScene{1}.faces = face;  
    planningScene{1}.faceNormals = faceNormals;
    

    obj.PlanningScene = planningScene;
        
end

%% INITIALISE SENSORS
function initialiseSensor(obj, sim, display)
    %INITIALISESENSOR Initialises an instance of the sensor class
    % Initialises an instance of the sensor class. If simulated, will
    % connect the human detection service to the computers webcam.
    % Otherwise will conect the human detection service to the kinect
    % camera.

    obj.Sensor = CustomSensor(sim);
    if sim
       obj.Sensor.initialiseHumanDetection(1, '/usb_cam/image_raw/compressed', '/Sensor/Human', display);
    else
       obj.Sensor.initialiseHumanDetection(1, '/camera/depth_registered/points', '/Sensor/Human', display);
       obj.Sensor.setupCamera('/camera/depth_registered/points')
    end 

end

%% INITIALISE CONTROLLER
function initialiseController(obj, sim)
    %INITIALISECONTROLLER Initialises an instance of the controller class
    
    obj.Controller = Controller(sim, obj.Robot, obj.PlanningScene);
end

%% GET TARGETS FROM SENSOR
function getTargets(obj)
    %GETTARGETS uses the initialised sensor to detect cracks in the rgbd
    %data, either from the kinect sensor or the stored ROSBAG
    
    crackData = obj.Sensor.detectCracks(true); % Detect and display cracks
    sel = input("Please select a crack to repair"); % wait for user input to select which crack to repair
    obj.Target = crackData{sel};  % Save selected crack coordinates as target

end

%% PLAN TRAJECTORY USING CONTROLLER
function planTrajectory(obj)
    %PLANTRAJECTORY uses the controller to plan a trajectory which passes
    %through the target points determiend from getTargets
    obj.Controller.makeTrajectory(double(obj.Target));

end

%% RUN TRAJECTORY USING CONTROLLER
function runTrajectory(obj)
    %RUNTRAJECTORY Connects the controller to the human sensor and begins
    % moving along the planned trajectory
   obj.Controller.connectHuman(obj.Sensor);
   obj.Controller.startMovement();
   
end 

%% ESTOP FUNCTION
function eStop(obj)
    %ESTOP Simulates an Estop that toggles a safety value. Once pressed,
    %will alternate the currently stored estop toggle value. Pressing the
    %eStop once will stop the robot, pressing it again will start the robot
    %on the condition that no humans are detected in the workspace. 
    if ~isempty(obj.Controller)
        display("eStop Pressed");
        obj.Controller.eStopSafe = ~obj.Controller.eStopSafe;
    end 
end 

    end
end

