classdef A2UR3class
    %class to contain robot properties and methods
    
    properties
        
        %robot model
        model
        %workspace
        workspace = [-5 5 -5 5 0 5]
        %name
        name = 'RoboWelder'
        %end effector point cloud
        cloud
        %log object
        log
        
    end
    
    methods
% UR3 methods
%construct object, initialise serial link and plot
function self = UR3(name, base, initialise, log)
    
    self.log = log;
    
    self.log.mlog = {self.log.DEBUG, class(self), 'Constructing UR3 object'};
            
     % Create the UR3 model mounted on a linear rail
    L1 = Link([pi    0           0        pi/2    1]); % PRISMATIC Link
    L2 = Link('a',0,        'd',0.1519, 'alpha',pi/2);
    L3 = Link('a',-0.24365, 'd',0,      'alpha',0);
    L4 = Link('a',-0.21325, 'd',0,      'alpha',0);
    L5 = Link('a',0,        'd',0.11235,'alpha',pi/2);
    L6 = Link('a',0,        'd',0.08535,'alpha',-pi/2);
    L7 = Link('a',0,        'd',0.0819, 'alpha',0);
    
    % Incorporate joint limits
    L1.qlim = [-0.8 0];
    L2.qlim = [-360 360]*pi/180;
    L3.qlim = [-360 360]*pi/180;
    L4.qlim = [-360 360]*pi/180;
    L5.qlim = [-360 360]*pi/180;
    L6.qlim = [-360 360]*pi/180;
    L7.qlim = [-360 360]*pi/180;
    
    % Offset to get into nice starting position
    L3.offset = -pi/2;
    L5.offset = -pi/2;      
    L1.offset = 0.4;
    
    % Set model and name property
    self.model = SerialLink([L1, L2, L3, L4, L5, L6, L7], 'name', name);
    self.model.base = base;
    self.name = name;
    
    % Get base into right orientation
    self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);

    % If initialise is true, plot the robot
    if initialise
        self.PlotAndColourRobot;
    end 
    
    self.log.mlog = {self.log.DEBUG, class(self), 'UR3 object constructed successfully'};
            
end    
 
%% PlotCloud
% Plot the robots possible positions, incorporating the retrictions from a
% bounding box. Bounding box in format [[x0, x1] [y0 y1] [z0 z1]. For no
% boundary at element, use -inf or inf.
function PlotCloud(self, stepsize, boundary)
    
    self.log.mlog = {self.log.DEBUG, class(self), "Generating point cloud"};

    % Generate angular/angular steps for joints
    angularSteps = 0:stepsize:2*pi;
    linearSteps = -0.8:stepsize:0;
    
    % Generate a matrix containing each combination of the generated steps
    % for each joint. (Last joint always set to 0 as it does not effect end
    % effector position).
    C = {linearSteps,angularSteps,angularSteps,angularSteps,angularSteps, angularSteps};
    D = C;
    [D{:}] = ndgrid(C{:});
    Z = cell2mat(cellfun(@(m)m(:),D,'uni',0));
    Z = [Z, zeros(length(Z),1)];
    
    % Get the position for each of the joint angles passed and remove axes
    % of length 1
    trCloud = self.model.fkine(Z);
    pointCloud = squeeze(trCloud(1:3,4,:));
    
    % Get x y and z elements of the point cloud
    x = pointCloud(1,:);
    y = pointCloud(2,:);
    z = pointCloud(3,:);
    
    % Restrict to elements that are within the boundary box
    boundedPoints = (z >= boundary(3,1) & z <= boundary(3,2) & y >= boundary(2,1) & y <= boundary(2,2) & x >= boundary(1,1) & x <= boundary(1,2));
    x = x(boundedPoints);
    y = y(boundedPoints);
    z = z(boundedPoints);

    % Form a convex hull out of the point cloud
    [k1, v1] = convhull(x,y,z);
    
    % Plot the hull
    self.cloud = trisurf(k1,x,y,z,'FaceColor','cyan');
    
    self.log.mlog = {self.log.DEBUG, class(self), ["Cloud of " , int2str(length(pointCloud)) , " points generated"]};
    self.log.mlog = {self.log.DEBUG, class(self), ["Workspace has approximate volume of " , int2str(v1) , " m^3"]};
    self.log.mlog = {self.log.DEBUG, class(self), "When finished viewing the cloud, call obj.DeleteCloud() to clear space"};

end 
        
%% DeleteCloud
% Remove the point cloud generated by PlotCloud
function DeleteCloud(self)
    
    try
        self.cloud.delete
        self.log.mlog = {self.log.DEBUG, class(self), "Cloud deleted"};
    catch
        self.log.mlog = {self.log.DEBUG, class(self), "No cloud to delete"};
    end 
    
end 

%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)
    
    self.log.mlog = {self.log.DEBUG, class(self), 'Plotting 3D UR3'};

    for linkIndex = 0:self.model.n

        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        
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
            self.log.mlog = {self.log.WARN, class(self), strcat('No vertex colors found in ' , 'LinUR3Link' , num2str(linkIndex) ,'.ply') };
            disp(ME_1);
            continue;
        end
    end
    
    self.log.mlog = {self.log.DEBUG, class(self), 'UR3 Plotted Successfully'};

    
end 

    end
end
