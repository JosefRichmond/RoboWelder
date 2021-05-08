classdef Sensor < handle
%SENSOR Node that handles all sensing applications of the robot
%   The Sensor class utilises input from an RGB-D camera to detect cracks
%   and monitor the workspace for humans. If a human is detected, the
%   Sensor will display a warning. This functionality is initiated by
%   calling obj.initialiseHumanDetection which allows you to specify the
%   monitoring rate. The crack detection is modelled after a ROS service,
%   and only occurs once called by the user via the detectCracks function.
    
properties
%% Properties Relating to Human Detection
humanSub % Subscriber to camera node
faceDetector % Face detector object
faceDetectorTimer % Timer control for face detection

%% Properties Relating to Crack Detection
cameraSub % 
cameraPub
end    

methods
%% Object Constructor

function obj = Sensor()
%SENSOR Construct an instance of this class
%   Does nothing but construct the object at the moment, could add in
%   initialisation here or some toggles for features like monitoring for
%   humans
end
%% Initialise Human Detection

function initialiseHumanDetection(obj,rate, cameraNode)
%INITIALISEHUMANDETECTION
%   Creates the face detection object, and sets up the subscriber to the
%   camera node. Then creates a timer, with a fixedRate mode of operation
%   that calls the face detection function at the specified rate. 
%   PARAMETERS
%       rate - Rate at which workspace is checked for faces (Hz)
%       cameraNode - Name of node sending camera footage
    
    % Create face detector and subscribers
    obj.faceDetector = vision.CascadeObjectDetector();
    obj.humanSub = rossubscriber(cameraNode);
    
    % cameraNode = '/usb_cam/image_raw/compressed' 

    % Initialise timer with given rate and set callback function to
    % detectFaces
    obj.faceDetectorTimer= timer('Period', 1/rate, 'ExecutionMode','fixedRate');
    obj.faceDetectorTimer.TimerFcn =  @(~, ~) obj.detectFaces;
    
    % Start timer
    start(obj.faceDetectorTimer);
end
%% Face Detection Callback Function  

function detectFaces(obj)
%DETECTFACES Detects faces from camera footage
%   Function that detects faces using camera footage from the specified
%   node saved in the humanSub property. Uses a faceDetection object saved
%   in the faceDetector property. Is only called through a timer and should
%   not be directly called by humans. Currently does not output anything,
%   but displays "Safe" or "DANGER" depending on whether there is a human
%   present in the workspace. This should be rectified to publish a true or
%   false which can be accessed and interpreted by the CORE class. 

    msg = obj.humanSub.receive(10);
    img = readImage(msg);
    loc = step(obj.faceDetector, img);
    if isempty(loc)
        display('Safe');
    else
        display('DANGER');
    end
end
%% Crack Detection Initialisation

function setupCamera(obj)
% SETUPCAMERA Sets up a fake camera publisher to replicate input from an
% RGB-D camera
%   Creates a publisher to simulate input from an RGB-D camera. Needs to be
%   edited to easily switch between simulated and real input

    nodeName = '/rgbd';
    obj.cameraPub = rospublisher(nodeName,'sensor_msgs/PointCloud2','DataFormat','struct');
end
%% RGB-D Data Request Function

function requestRGBD(obj)    
% REQUESTRGBD Function that emulates service calls from ROS.
%   When called, currently loads data from a .mat file containing RGB-D
%   data and publishes it via cameraPub. Only works for simulated input
%   right now. Will be revised when access to RGB-D camera is possible. 
   
    ptcloud2 = load('ptcloud2');
    ptcloud2 = ptcloud2.ptcloud2;
   
    msg = rosmessage(obj.cameraPub);
    msg.width = uint32(448);
    msg.height = uint32(448);
    msg.point_step = ptcloud2.PointStep;
    msg.row_step = ptcloud2.RowStep;
    msg.data = ptcloud2.Data;
    %msg.fields = struct(ptcloud2.Fields);

    send(obj.cameraPub,msg);
end
%% Crack Detection Function

function data = detectCracks(obj)
% DETECTCRACKS Uses a neural network to detect cracks
%   Currently performs image segmentation using a resnet trained via
%   transfer learning on a dataset of cracks. Once the image is segmented
%   into crack & not crack, the morphological function library is used to
%   'skeletonize' the crack regions and determine the centre path of the
%   crack. Each crack region is the split and xyz coordinates along the
%   path determined. Currently nothing is returned, this will be rectified
%   when access to RGB-D camera is possible. 
            
    dataRead = rossubscriber('/rgbd'); %change this to use variable for sub name
    obj.requestRGBD;
    data = receive(dataRead,10); %change timeout time to be variable
    
    % This bit here is dumb, accessing the saved point cloud message to get
    % field names
    ptCloud = data;
    ptcloud2 = load('ptcloud2');
    ptcloud2 = ptcloud2.ptcloud2;
    ptCloud.Fields = ptcloud2.Fields;

    % Reads the xyz and RGB data from the cloud
    xyz = readXYZ(ptCloud);
    rgb = readRGB(ptCloud);

    % Shapes back into an image & converts to correct data type
    img = reshape(rgb,448,448,3);
    img = uint8(img*255);
    
    % Load the saved network
    net = load('network');
    net = net.net;

    % Perform semantic segmentation to locate cracks
    C = semanticseg(img,net);
    bw = ones(448,448);
    bw(C == "Crack") = 1;
    bw(C == "NoCrack") = 0;
    
    % Binarize and find skeletons of cracks. Still need to remove branches
    bw = imbinarize(bw);
    skelImage = bwskel(bw, 'MinBranchLength', 10);
    
    % Plots the detected cracks
    [l1,l2] = bwlabel(skelImage);
    cData = {};
    for i =1:l2
        subplot(ceil((l2+1)/2),ceil((l2+1)/2),i+1)
        r1 = skelImage;
        r1(l1 ~= i) = 0;
        imshow(bwmorph(r1,'thicken',2));
        title("Crack Branch " + num2str(i))
        [x,y] = find(r1);
        cData{i} = [x,y];
        pbaspect([2,1,1])
    end
    
    subplot(ceil((l2+1)/2),ceil((l2+1)/2),1)
    imshow(bwmorph(skelImage,'thicken',2));
    title('Detected Cracks');
    pbaspect([2,1,1])
            
end
%% Object Destructor

function delete(obj)
    try
        obj.faceDetectorTimer.stop();
    catch
        disp('Could not shutdown timer')
    end
end 
    end
end

