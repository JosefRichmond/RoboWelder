function Restart(pubName)
%RESTART Summary of this function goes here
%   Detailed explanation goes here
sub = rossubscriber(pubName);% specifies the topic name, message type, and callback function for the subscriber.
display("RESTARTING")
humanDetected = false;
for i = 1:10
    msg = receive(sub,10);
    display("TRIAL " + num2str(i) + " OF " + num2str(10));
    if msg.Data == true
        humanDetected = true;
        break
    else
        display("SUCCESSFUL")
    end 
end 
if humanDetected
    display("HUMAN DETECTED, ABORTED RESTART");
else
    display("RESTART CONFIRMED");
end 
end

