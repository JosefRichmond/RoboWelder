%% This is used in ROS_Subscribers.m to illustrate how to use callback functions

classdef SimpleHandleClass < handle
   properties
      receiveProperty
      editProperty
   end
   methods
      function obj = SimpleHandleClass(r)
         if nargin > 0
            obj.receiveProperty = r;
         end
      end
   end
end

