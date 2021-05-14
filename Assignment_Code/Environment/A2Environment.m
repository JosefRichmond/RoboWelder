classdef A2Environment < handle
    %Custom class to define our A2 environment
   
    
    properties
        %objects structure
        objects
        %robot structure
        robot
        %workspace structure
        workspace = [-5 5 -5 5 0 5]
        
        
    end
    
    methods
%% Environment - construct object, set workspace
        function [self,f,v] = A2Environment()
            
            %self.workspace = workspace;
            [f,v] = self.LoadEquipment();
            %self.LoadRobot();
            axis equal
        end

%% Robot - load into environment
        function LoadRobot(self)
            
            %load the robot
            self.robot.UR3 = UR3;
            %plot and colour robot
            UR3.PlotAndColourRobot
        end
        
%% Objects - load surrounding objects into environment
function [f,v] = LoadEquipment(self)
        [f,v,data] = plyread('roboenviro.ply', 'tri');
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        enviroMesh = trisurf(f,v(:,1),v(:,2), v(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
        end

    end
end

