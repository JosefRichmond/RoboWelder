try
    rosinit
end 
serviceServer = rossvcserver("/test","nav_msgs/GetMap",@serviceCallback);
%% 

testclient = rossvcclient('/test');
testreq = rosmessage(testclient);
%% 

testresp = call(testclient,testreq,'Timeout',10);

%% 
rosshutdown


%% 
function response = serviceCallback(src,reqMsg,defaultRespMsg)

response = defaultRespMsg

end 