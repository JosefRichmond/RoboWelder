try 
    rosinit
end 
mySensor = Sensor();
try
    s = rossubscriber('/usb_cam/image_raw/compressed',@mySensor.detectFaces, 'BufferSize',1);
catch
    clear('s')
end 
