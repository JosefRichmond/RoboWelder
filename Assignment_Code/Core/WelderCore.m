classdef WelderCore < handle
    %CORE Summary of this class goes here
    %   Detailed explanation goes here
    
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
function obj = WelderCore(sim)
    %CORE Construct an instance of this class
    %   Detailed explanation goes here
    obj.sim = sim;
    try
        if sim
            rosinit
        else 
            rosinit('192.168.0.253')
        end 
    end 

end

%%
function initialiseRobot(obj)
%     obj.Robot = UR5; 
    obj.Robot = A2_UR3(false,true);
    
end 

%%
function initialiseEnvironment(obj, loadEnvironment)
    %METHOD1 Summary of this method goes here
    %   Detailed explanation goes here
    
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
%     
%     planningScene{1}.vertex = vertex;
%     planningScene{1}.faces = face;  
%     planningScene{1}.faceNormals = faceNormals;
    
<<<<<<< HEAD
    planningScene{1}.vertex = vertex;
    planningScene{1}.faces = face;  
    planningScene{1}.faceNormals = faceNormals;
    
=======
>>>>>>> 0300fcf2838dcb11ff3b8b5495396e13a3a3edb8
    obj.PlanningScene = planningScene;
        
end
%%
function initialiseSensor(obj, sim, display)
    
<<<<<<< HEAD
    obj.Sensor = CustomSensor(sim);
    if obj.sim
       obj.Sensor.initialiseHumanDetection(1, '/usb_cam/image_raw/compressed', '/Sensor/Human', display);
    else
       obj.Sensor.initialiseHumanDetection(1, '/camera/depth_registered/points', '/Sensor/Human', display);
       obj.Sensor.setupCamera('/camera/depth_registered/points')
    end 
=======
    obj.Sensor = CustomSensor(true);
    obj.Sensor.initialiseHumanDetection(1, '/camera/rgb/image_raw', '/Sensor/Human', display);

%     if sim
%      obj.Sensor.initialiseHumanDetection(1, '/usb_cam/image_raw/compressed', '/Sensor/Human', display);
%     else
%        obj.Sensor.initialiseHumanDetection(1, '/camera/depth_registered/points', '/Sensor/Human', display);
% %        obj.Sensor.setupCamera('/camera/depth_registered/points')
%     end 
>>>>>>> 0300fcf2838dcb11ff3b8b5495396e13a3a3edb8
    
end

%%
function initialiseController(obj, sim)
    
    obj.Controller = Controller(sim, obj.Robot, obj.PlanningScene);
    
end

%%
function getTargets(obj)
    
    crackData = obj.Sensor.detectCracks(true);
    sel = input("Please select a crack to repair");
    obj.Target = crackData{sel}; 

end

%%
function planTrajectory(obj)
    
    obj.Controller.makeTrajectory(double(obj.Target));

end

%%
function runTrajectory(obj)
    
   obj.Controller.connectHuman(obj.Sensor);
   obj.Controller.startMovement();
   
end 
%%
function eStop(obj)
    if ~isempty(obj.Controller)
        display("eStop Pressed");
        obj.Controller.eStopSafe = ~obj.Controller.eStopSafe;
    end 
end 

    end
end

