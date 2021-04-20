function goodCallBack(~,message,simpleClass)
    
    % By passing in a class object, we can access properties
    rVal = simpleClass.receiveProperty;
    
    %And edit properties
    simpleClass.editProperty = randi([1,100],[1,1]);

end 
