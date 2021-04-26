function goodCallBack(~,message,simpleClass)
% This function is used as a call back that edits a class property rather than a global variable as is done in the online tutorial

    % By passing in a class object, we can access properties
    rVal = simpleClass.receiveProperty;
    
    %And edit properties
    simpleClass.editProperty = randi([1,100],[1,1]);

end 
