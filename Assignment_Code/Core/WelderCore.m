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
    
    planningScene{1}.vertex = vertex;
    planningScene{1}.faces = face;  
    planningScene{1}.faceNormals = faceNormals;
    
    obj.PlanningScene = planningScene;
        
%     centerpnt = [0,0,-1.01];
%     side = 2;
%     plotOptions.plotFaces = true;
%     [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,[0,0,1],plotOptions);
% 
      
%     planningScene{1}.vertex = vertex;
%     planningScene{1}.faces = faces;
%     planningScene{1}.faceNormals = faceNormals; 

%     centerpnt = [0,0.94,0.5];
%     side = 1;
%     [v1,v2,v3] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,[1,0,0],plotOptions);
% 
%     planningScene{2}.vertex = v1;
%     planningScene{2}.faces = v2;
%     planningScene{2}.faceNormals = v3;
%     
%     obj.PlanningScene = planningScene;
    % 
end
%%
function initialiseSensor(obj, sim, display)
    
    obj.Sensor = CustomSensor(sim);
    if obj.sim
       obj.Sensor.initialiseHumanDetection(1, '/usb_cam/image_raw/compressed', '/Sensor/Human', display);
    else
       obj.Sensor.initialiseHumanDetection(1, '/camera/depth_registered/points', '/Sensor/Human', display);
       obj.Sensor.setupCamera('/camera/depth_registered/points')
    end 
    
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
    
%     obj.Controller.makeTrajectory(double(obj.Target(1:10:length(obj.Target),:)));
    obj.Controller.makeTrajectory(double(obj.Target));

end

%%
function runTrajectory(obj)
    
   obj.Controller.connectHuman(obj.Sensor);
   obj.Controller.startMovement();
   
end 

    end
end

