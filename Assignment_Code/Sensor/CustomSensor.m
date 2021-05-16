classdef CustomSensor < handle
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
humanPub % Publishes true/false info on human detection
faceDetector % Face detector object
faceDetectorTimer % Timer control for face detection
showFig

%% Properties Relating to Crack Detection
cameraSub % 
cameraPub
cameraPubName
sim

end    

methods
%% Object Constructor
function obj = CustomSensor(Sim)
%SENSOR Construct an instance of this class
%   Does nothing but construct the object at the moment and save the
%   boolean indicating simulation or not

% Kinect pub = /camera/depth_registered/points
% /usb_cam/image_raw/compressed
% Kinect launch = roslaunch freenect_launch freenect.launch depth_registration:=true

obj.sim = Sim;

end

%% Initialise Human Detection
function initialiseHumanDetection(obj,rate, cameraNode, pubName, displayFace)
%INITIALISEHUMANDETECTION
%   Creates the face detection object, and sets up the subscriber to the
%   camera node. Then creates a timer, with a fixedRate mode of operation
%   that calls the face detection function at the specified rate. 
    
    % Create face detector and subscribers
    obj.faceDetector = vision.CascadeObjectDetector();
    obj.humanSub = rossubscriber(cameraNode);
    
    % Initialise timer with given rate and set callback function to
    % detectFaces
    obj.faceDetectorTimer= timer('Period', 1/rate, 'ExecutionMode','fixedRate');
    obj.faceDetectorTimer.TimerFcn =  {@obj.detectFaces, displayFace};
    
    % Create publisher
    obj.humanPub = rospublisher(pubName,'std_msgs/String','DataFormat','struct');

    % Start timer
    start(obj.faceDetectorTimer);
    
end

%% Face Detection Callback Function  
function detectFaces(obj,~, ~, display)
%DETECTFACES Detects faces from camera footage
%   Function that detects faces using camera footage from the specified
%   node saved in the humanSub property. Uses a faceDetection object saved
%   in the faceDetector property. Is only called through a timer and should
%   not be directly called by humans. Currently publishes a true/false
%   to the /sensor/human topic indicating if a human has been detected or
%   not. 
   
    % Receives a message from the camera node
    msg = obj.humanSub.receive(10);
    img = readImage(msg);
    
    % Detects faces and returns location of corners of a square that
    % surrounds that face
    loc = step(obj.faceDetector, img);
    
    % Creates new message
    outMsg = rosmessage(obj.humanPub);
    
    % True if face detected, false otherwise
    if isempty(loc)
        outMsg.data = 'false';
    else
        outMsg.data = 'true';
    end
    
    % Publish the true/false message
    send(obj.humanPub,outMsg);
    
    % Display the camera feed w/ faces surrounded by bounding boxes
    if display
        
        if isempty(obj.showFig)
        figure()
        imshow(img);
        obj.showFig = axes;
        end 
        
        img = insertShape(img, 'Rectangle', loc);
        imshow(img, 'Parent' , obj.showFig);
        
    end 
    
end

%% Deactivate Human Detection
function deactivateHumanDetection(obj)
%DEACTIVATEHUMANDETECTION Stops face detection process
%   Stops face detection process by deleting the timer triggering the
%   detection callback

    try
        obj.faceDetectorTimer.stop();
        obj.showFig = [];
    catch
        disp('Could not shutdown face detection routine, check timer still exists')
    end
end 

%% Crack Detection Initialisation
function setupCamera(obj, nodeName)
% SETUPCAMERA Sets up a fake camera publisher to replicate input from an
% RGB-D camera
%   Creates a publisher to simulate input from an RGB-D camera. Needs to be
%   edited to easily switch between simulated and real input
    obj.cameraPubName = nodeName;
    obj.cameraSub = rossubscriber(nodeName);  
end

%% Crack Detection Function
function cData = detectCracks(obj, displayCracks)
% DETECTCRACKS Uses a neural network to detect cracks
%   Currently performs image segmentation using a resnet trained via
%   transfer learning on a dataset of cracks. Once the image is segmented
%   into crack & not crack, the morphological function library is used to
%   'skeletonize' the crack regions and determine the centre path of the
%   crack. Each crack region is the split and xyz coordinates along the
%   path determined.
          
    if obj.sim
        bag = rosbag('2021-05-12-22-20-29.bag');
        data = bag.readMessages{1};
    else
        data = receive(obj.cameraSub,10); %change timeout time to be variable
    end 
    
    % This bit here is dumb, accessing the saved point cloud message to get
    % field names
    ptCloud = data;

    % Reads the xyz and RGB data from the cloud
    xyz = readXYZ(ptCloud);
    rgb = readRGB(ptCloud);

    % Shapes back into an image & converts to correct data type
    img = reshape(rgb,data.Width, data.Height,3);
    img = uint8(img*255);
    
    % Load the saved network
    net = load('network');
    net = net.net;

    % Perform semantic segmentation to locate cracks
    C = semanticseg(img,net);
    bw = ones(data.Width,data.Height);
    bw(C == "Crack") = 1;
    bw(C == "NoCrack") = 0;
    
    % Binarize and find skeletons of cracks. Still need to remove branches
    bw = imbinarize(bw);
    skelImage = bwskel(bw, 'MinBranchLength', 10);
    
    % Plots the detected cracks
    [l1,l2] = bwlabel(skelImage);
    cData = {};
    
    % Overlay the detected cracks onto the original images
    figure()
    overlay = rgb2gray(img);
    overlay(skelImage) = 255;
    imshow(overlay)
 
    % Converts the xyz coordinates into 4x4 transforms
    pos = arrayfun(@transl, xyz(:,1), xyz(:,2), xyz(:,3), 'UniformOutput', false);
    
    % Rotate and translate to get data into robot base frame
    rotpos = cellfun(@(p) trotx(-pi/2)*transl(0,-0.5,0)*p,pos,'un',0);
    rotpostrim = cellfun(@(rotposl) rotposl(1:3,4)',rotpos,'un',0);
    rotpostrimmat = cell2mat(rotpostrim);
    
    % If using the simulated data, just centre on robot base frame to make
    % things easier to debug and visualise
    if obj.sim
        rotpostrimmat = rotpostrimmat + [0.2,-0.3,-0.1];
    end 
    
    % For each detected crack find the major points and append to a cell
    % array. If display is requested, display the skeleton image of the
    % cracks
    figure()
    for i =1:l2
        r1 = skelImage;
        r1(l1 ~= i) = 0;
        eP = bwmorph(r1,'endpoints');
        rgb(r1,:) = repmat([1, 0, 0],  length(rgb(r1,:)), 1);
        if displayCracks
            subplot(2,ceil((l2)/2),i)
            imshow(r1);
            title("Crack Branch " + num2str(i))
        end 
        cData{i} = rotpostrimmat(eP,:);
    end
    
    % Plot the robot and the RGB-D data
    figure()
    robot = A2_UR3(false,true);
    robot.model.animate(deg2rad([-180,0,0,0,-180,0]))
    hold on
    trplot(transl(0,0,0))  
    scatter3(rotpostrimmat(:,1),rotpostrimmat(:,2) ,rotpostrimmat(:,3)  , 1 , rgb);
    

end

%% Object Destructor
function delete(obj)
    % Attempts to stop the face detection service and resets the output
    % plot to empty
    try
        obj.faceDetectorTimer.stop();
        obj.showFig = [];
    catch
        disp('Could not shutdown timer')
    end
end 
    end
end

