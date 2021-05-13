%% ASSIGNMENT #2 UR3 CLASS
% Main % Robotics 41013 Lab Assignment 2
% Contributors: Alistair Higgins 12934600
%               Reece Holmewood 12875629
%               Josef Richmond 12875860

classdef A2_UR3 < handle
    properties
        %> Robot model
        model;
        
        %> Workspace dimensions
        workspace = [-2 2 -2 2 -0.3 2];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;     
        
        %> Plot robot flag
        plotRobot = false;
    end
    
    methods %% Class for UR5 robot simulation
function self = A2_UR3(useGripper, plotRobot)
    
    if nargin < 1
        useGripper = false;
        plotRobot = false;
    end
    
    self.useGripper = useGripper;
            
    self.GetUR3Robot();

    if plotRobot == true
        self.PlotAndColourRobot();
    end

end

%% GetUR5Robot
% Given a name (optional), create and return a UR5 robot model
function GetUR3Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    L1 = Link('d',0.1519, 'a',0,        'alpha',pi/2, 'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L2 = Link('d',0,      'a',-0.24365, 'alpha',0,    'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]);    % -pi/2
    L3 = Link('d',0,      'a',-0.21325, 'alpha',0,    'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
    L4 = Link('d',0.11235,'a',0,        'alpha',pi/2, 'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);  % -pi/2
    L5 = Link('d',0.08535,'a',0,        'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
    L6 = Link('d',0.0819, 'a',0,        'alpha',0,    'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

    self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 1:self.model.n

        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['/home/reeceh/git/RoboWelder/PlyFiles/UR3Decimated/','UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
        
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end