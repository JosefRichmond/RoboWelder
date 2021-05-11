%% Exercise 2
% 2.1 Download the PDF of the robot from UTSOnline

% 2.2 Determine the D&H parameters based upon the link measurements on the PDF
% & 2.3 Determine and include the joint limits in your model
L1=Link('alpha',-pi/2,'a',0.180, 'd',0.475, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
L2=Link('alpha',0,'a',0.385, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
L3=Link('alpha',pi/2,'a',-0.100, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
L4=Link('alpha',-pi/2,'a',0, 'd',0.329+0.116, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
L5=Link('alpha',pi/2,'a',0, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
L6=Link('alpha',0,'a',0, 'd',0.09, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);
    
densoRobot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Denso VM6083G');
densoRobot.name = 'Denso VM6083G';
% Use glyphs to draw robot, don't display the name
densoRobot.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist'}; 
q = [0,pi/2,0,0,0,0];
densoRobot.plot(q);