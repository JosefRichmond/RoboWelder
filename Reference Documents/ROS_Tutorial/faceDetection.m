try
    rosinit;
end 
camSub = rossubscriber('/usb_cam/image_raw/compressed');
faceDetector = vision.CascadeObjectDetector();
while(1)
    msg = camSub.receive(10);
    img = readImage(msg);
    loc = step(faceDetector, img);
    det = insertShape(img,'Rectangle',loc);
    imshow(det)
end 

