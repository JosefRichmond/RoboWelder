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

