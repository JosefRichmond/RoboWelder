function Simple_Movement(point, client, goal, jointStateSubscriber,durationSeconds)

%Function handles the input of a Q-Matrix and sends data to the robot 
numPoints = size(point,1);
timePerStep = durationSeconds/numPoints;
counter = 1;

    while counter < numPoints
        % Determine current state
        currJointState = (jointStateSubscriber.LatestMessage.Position)';
        currJointState = [currJointState(3:-1:1),currJointState(4:6)];
        %currFeedback = (feedBackSubscriber.LatestMessage.Feedback)
        %currStatus = (feedBackSubscriber.LatestMessage.Status)
        
        pointCheck = abs(currJointState-point(counter,:));
        pcMax = max(pointCheck);
%          if counter == 1
%             % Set the start of next motion to current state
%             startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%             startJointSend.Positions = currJointState;
%             %startJointSend.Velocities = [pi/9 pi/9 pi/9 pi/9 pi/9 pi/9];
%             startJointSend.TimeFromStart = rosduration(0.05);  
%       
%             endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');  
%             endJointSend.Positions = point(counter,:);
%             %endJointSend.Velocities = [pi/9 pi/9 pi/9 pi/9 pi/9 pi/9];
%             endJointSend.TimeFromStart = rosduration(timePerStep);
%     
%             goal.Trajectory.Points = [startJointSend; endJointSend];
%         
%             goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(1);
%             sendGoal(client,goal);
%             
%             disp("Pos 1 sent")
%             counter = counter +1;
%             
%          end
         if pcMax < 0.001
        
            counter = counter+1;
            % Set the start of next motion to current state
            startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            startJointSend.Positions = currJointState;
            %startJointSend.Velocities = [pi/9 pi/9 pi/9 pi/9 pi/9 pi/9];
            startJointSend.TimeFromStart = rosduration(0.05);  
      
            endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');  
            endJointSend.Positions = point(counter,:);
            %endJointSend.Velocities = [pi/9 pi/9 pi/9 pi/9 pi/9 pi/9];
            endJointSend.TimeFromStart = rosduration(timePerStep);
    
            goal.Trajectory.Points = [startJointSend; endJointSend];
        
            goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(1);
            disp("Pos sent");
            sendGoal(client,goal);
        
         end
    
    end
end