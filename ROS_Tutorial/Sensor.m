classdef Sensor < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        humanPub
        cameraSub
        faceDetector
    end
    
    methods
        function obj = Sensor()
            %UNTITLED4 Construct an instance of this class
            %   Detailed explanation goes here
            obj.faceDetector = vision.CascadeObjectDetector();
            
        end
        
        function detectFaces( obj, ~, msg)
%             faceDetector = vision.CascadeObjectDetector();
            img = readImage(msg);
            loc = step(obj.faceDetector, img);
            if isempty(loc)
                display('Safe');
            else 
                display('DANGER');
            end 
%             det = insertShape(img,'Rectangle',loc);
%             imshow(img);
        end
    end
end

