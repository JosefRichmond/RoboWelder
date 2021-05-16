%% Test and Training Dataset locations

train_img_dir = fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","train","images");
train_msk_dir =  fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","train","masks");

test_img_dir = fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","val","images");
test_msk_dir =  fullfile("/", "media", "josef", "UUI","crack_segmentation_dataset","val","masks");

%%

classNames=  ["Crack" "NoCrack"];
pixelLabelID = [255 0];

%% 

train_img = imageDatastore(train_img_dir);
train_msk = pixelLabelDatastore(train_msk_dir, classNames, pixelLabelID);

test_img = imageDatastore(test_img_dir);
test_msk = pixelLabelDatastore(test_msk_dir, classNames, pixelLabelID);


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


%% Train Network

[net, info] = trainNetwork(dsTrain,lgraph,options);


