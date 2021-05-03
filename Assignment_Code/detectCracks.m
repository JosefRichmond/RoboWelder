function cData = detectCracks(img, net)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

C = semanticseg(img,net);
B = labeloverlay(img,C);
bw = ones(448,448);
bw(C == "Crack") = 1;
bw(C == "NoCrack") = 0;
bw = imbinarize(bw);
skelImage = bwskel(bw, 'MinBranchLength', 10); 

[l1,l2] = bwlabel(skelImage);

cData = {};

for i =1:l2
    subplot(ceil((l2+1)/2),ceil((l2+1)/2),i+1)
%     subplot(ceil((l2+1)),1,i+1)
    r1 = skelImage;
    r1(l1 ~= i) = 0;
    imshow(bwmorph(r1,'thicken',2));
    title("Crack Branch " + num2str(i))
    [x,y] = find(r1);
    cData{i} = [x,y];
    pbaspect([2,1,1])
end 

subplot(ceil((l2+1)/2),ceil((l2+1)/2),1)
% subplot(ceil((l2+1)),1,1)
imshow(bwmorph(skelImage,'thicken',2));
title('Detected Cracks');
pbaspect([2,1,1])
end
