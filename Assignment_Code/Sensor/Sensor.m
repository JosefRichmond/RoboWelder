classdef Sensor < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        humanPub
        cameraSub
        faceDetector
        serviceServer
        cameraPub
    end
    
    methods
        
        function obj = Sensor()
            %UNTITLED4 Construct an instance of this class
            %   Detailed explanation goes here
            obj.faceDetector = vision.CascadeObjectDetector();
            obj.serviceServer = rossvcserver("/test","roscpp/Empty",@obj.sendPhoto)
        end
        
        function detectFaces( obj, ~, msg)
%             faceDetector = vision.CascadeObjectDetector();
            img = readImage(msg);
            size(img)
            loc = step(obj.faceDetector, img);
            if isempty(loc)
                display('Safe');
            else 
                display('DANGER');
            end 
%             det = insertShape(img,'Rectangle',loc);
%             imshow(img);
        end
        
        function setupCamera(obj)
            nodeName = '/rgbd';
            obj.cameraPub = rospublisher(nodeName,'sensor_msgs/PointCloud2','DataFormat','struct');      
        end 
        
        function requestCamera(obj)

                ptcloud2 = load('ptcloud2'); 
                ptcloud2 = ptcloud2.ptcloud2;

                msg = rosmessage(obj.cameraPub);
                msg.width = uint32(448);
                msg.height = uint32(448);
                msg.point_step = ptcloud2.PointStep;
                msg.row_step = ptcloud2.RowStep;
                msg.data = ptcloud2.Data;
%                 msg.fields = struct(ptcloud2.Fields);

                send(obj.cameraPub,msg);
        end 
        
        function data = detectCracks(obj)
                       
            dataRead = rossubscriber('/rgbd'); %change this to use variable for sub name
            obj.requestCamera;
            data = receive(dataRead,10); %change timeout time to be variable
            
            ptCloud = data;
            ptcloud2 = load('ptcloud2'); 
            ptcloud2 = ptcloud2.ptcloud2;
            ptCloud.Fields = ptcloud2.Fields;
            
            xyz = readXYZ(ptCloud);
            rgb = readRGB(ptCloud);

            i = 1;
            img = reshape(rgb,448,448,3);
            img = uint8(img*255);
            net = load('network');
            net = net.net
            
            C = semanticseg(img,net);
            B = labeloverlay(img,C);
            bw = ones(448,448);
            bw(C == "Crack") = 1;
            bw(C == "NoCrack") = 0;
            bw = imbinarize(bw);
            skelImage = bwskel(bw, 'MinBranchLength', 10); 

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
        
    end
end

