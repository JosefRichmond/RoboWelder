%% Test and Training Dataset locations

train_img_dir = fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","train","images");
train_msk_dir =  fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","train","masks");

test_img_dir = fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","val","images");
test_msk_dir =  fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","val","masks");

% fGhttps://www.mathworks.com/help/vision/ug/semantic-segmentation-using-deep-learning.html

%% Test and Training Datasets
% 
train_img = imageDatastore(train_img_dir);
train_msk = imageDatastore(train_msk_dir);

test_img = imageDatastore(test_img_dir);
test_msk = imageDatastore(test_msk_dir);


%%

classNames=  ["Crack" "NoCrack"];
pixelLabelID = [255 0];

%% 

train_img = imageDatastore(train_img_dir);
train_msk = pixelLabelDatastore(train_msk_dir, classNames, pixelLabelID);

test_img = imageDatastore(test_img_dir);
test_msk = pixelLabelDatastore(test_msk_dir, classNames, pixelLabelID);

%%
% 
% for i =1:500
%     img = readimage(train_msk,i);
%     unique(img)
% end 

%% Load an image & overlay its mask

I = readimage(test_img,1);
C = readimage(test_msk,1);
B = labeloverlay(I,C );
imshow(B)

%%

buildingMask = C == 'NoCrack';

figure
imshowpair(I, buildingMask,'montage')

%% Get statistics on class weights

pxds = pixelLabelDatastore(train_msk_dir,classNames,pixelLabelID);
tbl = countEachLabel(pxds)

imageFreq = tbl.PixelCount ./ tbl.ImagePixelCount;
classWeights = median(imageFreq) ./ imageFreq

%% Create new layers to enable transfer learning

% Specify the size of the images;
imageSize = size(I);

% Specify the number of classes.
numClasses = numel(classNames);

% Create DeepLab v3+.
lgraph = deeplabv3plusLayers(imageSize, numClasses, "resnet18");

%% Specify class weights 

pxLayer = pixelClassificationLayer('Name','labels','Classes',tbl.Name,'ClassWeights',classWeights);
lgraph = replaceLayer(lgraph,"classification",pxLayer);

%% Training Options

% Define validation data.
dsVal = combine(test_img,test_msk);

% Define training options. 
options = trainingOptions('sgdm', ...
    'LearnRateSchedule','piecewise',...
    'LearnRateDropPeriod',10,...
    'LearnRateDropFactor',0.3,...
    'Momentum',0.9, ...
    'InitialLearnRate',1e-3, ...
    'L2Regularization',0.005, ...
    'ValidationData',dsVal,...
    'MaxEpochs',30, ...  
    'MiniBatchSize',10, ...
    'ExecutionEnvironment', 'auto',...
    'Shuffle','every-epoch', ...
    'CheckpointPath', tempdir, ...
    'VerboseFrequency',1,...
    'Plots','training-progress',...
    'ValidationPatience', 4);

%% Augment Training Data

dsTrain = combine(train_img, train_msk);
dsTrain = transform(dsTrain, @(data)blk(data,1));
%  xTrans = [-10 10];
% yTrans = [-10 10];
% dsTrain = transform(dsTrain, @(data)augmentImageAndLabel(data,xTrans,yTrans));

%% Train Network

[net, info] = trainNetwork(dsTrain,lgraph,options);

%% Test Network
% subplot(2,2,1)
net = load('network');
net = net.net

I = readimage(test_img,2);
C = semanticseg(I,net);
B = labeloverlay(I,C);
bw = ones(448,448);
bw(C == "Crack") = 1;
bw(C == "NoCrack") = 0;
bw = imbinarize(bw);
skelImage = bwskel(bw, 'MinBranchLength', 10); % First guess.  Strangely bwskel() doesn't seem to have any short spurs, no matter what MinBranchLength is.
% imshow(skelImage)

[l1,l2] = bwlabel(skelImage);

for i =1:l2
    subplot(1,l2+1,i+1)
    r1 = skelImage;
    r1(l1 ~= i) = 0;
    imshow(r1);
end 

subplot(1,l2+1,1)
imshow(skelImage);
% 
% subplot(2,2,2);
% r1 = skelImage;
% r1(l1 ~= 1) = 0;
% binaryImage = bwmorph(bw, 'skel', inf');
% measurements = regionprops(binaryImage, 'Area')


%% 

figure()% I = readimage(test_img,2);
I =cat(3,skelImage,skelImage*5,skelImage*10);
C = semanticseg(I,net);
B = labeloverlay(I,C);
bw = ones(448,448);
bw(C == "Crack") = 1;
bw(C == "NoCrack") = 0;
bw = imbinarize(bw);
imshow(B)
% skelImage = bwskel(bw, 'MinBranchLength', 10); % First guess.  Strangely bwskel() doesn't seem to have any short spurs, no matter what MinBranchLength is.
% imshow(skelImage)
%%

function out = blk(data, l)
label = data{2};
label(isundefined(label)) = "Crack";
out = {data{1},label};
end 