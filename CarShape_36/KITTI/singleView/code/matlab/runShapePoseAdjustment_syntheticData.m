%% To run the single view shape and pose adjustment pipeline with ground plane formulation.
% Sarthak Sharma


%%
clc; clear all; close all;
% Declare/ read the global variables
% Camera calibratin matrix
K = [721.537700 0.000000 609.559300; 0.000000 721.537700 172.854000; 0 0 1];

% Load the meanshape and the basis vectors.
meanshape = importdata('../../data/meanShape.txt');
vectors = importdata('../../data/vectors.txt');
% visualizeWireframe3D(meanshape',' :mean of the car');pause(2)
% Average H, W ,L of the car in metres.
avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Keypoint lookup table for binary weight.
kp_lookup= load('../../data/kpLookup_azimuth.mat');kp_lookup = kp_lookup.kpLookup;

% Load the keypoint and the viewpoint predictions.
keypoints = importdata('../../data/results_KP.txt');
az_gt = 30; offset = 20;
pitch_gt =17;
az = az_gt + offset;

% First scale the wireframe/vectors anisotropically.
l = abs(max(meanshape(:,1)) - min(meanshape(:,1))); % Along the X direction.
w = abs(max(meanshape(:,3))- min(meanshape(:,3))); % Along the Z direction.
h =  abs(max(meanshape(:,2)) - min(meanshape(:,2))); % Along the Y direction.

sz= avgCarWidth / (1.1*w);
sy = avgCarHeight / h;
sx = avgCarLength / l;

meanshapeScaled = diag([sx,sy,sz])*(meanshape');


for i=1:size(vectors,1)
    vectorsScaled(i,:) = reshape((diag([sx,sy,sz])*reshape(vectors(i,:),3,36)),1,3*36);
end

% Apply the transformations to bring from shape net object frame to KITTI
% frame.
R = rotz(180)*roty(90); % Bring from shape net object frame to KITTI frame.
% R= rotx(pitch_gt+5)*R;% Apply pitch
R = roty(az)*R; % Apply azimuth

meanshapeRotatedScaled = R * (meanshapeScaled);
for j=1:size(vectors,1)
    vectorsRotatedScaled(j,:) =reshape((R*reshape(vectorsScaled(j,:),3,36)),1,3*36);
end

% Translate the car

% Ours
bbox = [558.6644  134.2858   74.0249   63.0642]; % x y w h
plane = [-0.0116 0.9500 0.3122 10.0739];
b = [bbox(1) + (bbox(3)-1)/2; bbox(2)+bbox(4)-1; 1];
n = plane(1,1:3); d = plane(1,4);

alpha = acos( dot(inv(K)*b,n) /( norm(inv(K)*b)*norm(n)) );
lengthOfTrans = sec(alpha)*(d);
X3d_cameraHeight = lengthOfTrans* (inv(K)*b);
t_gp = rotx(acosd(n(2)))*[0,-avgCarHeight/2,avgCarLength/2]';
X3d_cameraHeight= X3d_cameraHeight + t_gp;


% % Mobile eye
% rec = [ 558.6644  134.2858   74.0249   63.0642];
% b=[rec(1) + (rec(3)-1)/2 ;rec(2)+rec(4)-1 ;1];
% n=[0;-1;0]';d=1.7;
% nr = K\b;
% dr = n*(K\b);
% X3d_mobileEye = -1* (d/dr)*nr + [0,-avgCarHeight/2, avgCarLength/2]';    

X3d_gt = [-0.5000   -0.4825   30.0000]';

X3d = X3d_gt;
% X3d = X3d_cameraHeight;
% X3d = X3d_mobileEye;
meanshapeTranslatedRotatedScaled = meanshapeRotatedScaled + repmat(X3d,1,36);

carCenterViaShape = mean(meanshapeTranslatedRotatedScaled');

% FOR VISUALIZATION OF THE INITIAL WIREFRAME
w2d= K*meanshapeTranslatedRotatedScaled;
w2d(1,:) = w2d(1,:)./w2d(3,:);
w2d(2,:) = w2d(2,:)./w2d(3,:);
visualizeSynScene(w2d,keypoints,' Initial Scene');
pause(3)
% visualizeWireframe3D(meanshapeTranslatedRotatedScaled,' :initital wireframe');
lambdas=[0.0208364480250510,0.00967136192730910,0.00717094319071054,0.00565548391744564,0.00467453361084465,0.00329643528181894,0.00209592766309844,0.00163342705234948,0.00104417262030107,0.000949492925761423,0.000805245112797721,0.000794819762941728,0.000738459491936479,0.000637465648951683,0.000545710387292436,0.000530415743589159,0.000430982248425289,0.000383471123360310,0.000364060566388879,0.000335845449738752,0.000293254266498406,0.000280809402233219,0.000253979011166301,0.000250439803923838,0.000239639422099641,0.000227739460767099,0.000212634691872905,0.000201197758665930,0.000191438687798717,0.000158787716884223,0.000153031995700756,0.000137391994149763,0.000122187478151364,0.000116773767656378,0.000105296101573683,9.79367791848786e-05,9.60947836203073e-05,8.04996641937673e-05,7.61223597074321e-05,6.60262537278197e-05,5.68724957185047e-05,5.44969447067057e-05];
% lambdas=rand(1,size(vectors,1));
%% Pose adjustment
numEigVec = size(vectors,1);
write_inp_pose_syn(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,keypoints,meanshapeRotatedScaled,vectorsRotatedScaled(1:numEigVec,:),az,kp_lookup,-1*n,d,lambdas(1:numEigVec));

system('./../../build/singleViewPoseAdjuster');
W_p = importdata('../../poseAndShapeIntermediate.txt');
% visualizeWireframe3D(W_p',' : after pose adjustment')
pause(3)
W_2d = K*W_p';
W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
visualizeSynScene(W_2d,keypoints,' After Pose');
pause(3)

%% Shape adjustment

write_inp_shape_syn(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,keypoints,meanshapeRotatedScaled,vectorsRotatedScaled(1:numEigVec,:),az,kp_lookup,lambdas(1:numEigVec));
system('./../../build/singleViewShapeAdjuster');
W_sh = importdata('../../ceres_output_singleViewShapeAdjuster.txt');
lambdasAfterShapeAdjustment = importdata('../../lambdasAfterShape.txt');

% visualizeWireframe3D(W_sh',' : after shape adjustment');
pause(3)
W_2d = K*W_sh';
W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
visualizeSynScene(W_2d,keypoints,' After Shape');

%% Error plots

% plot the lambdas
figure;
plot(lambdasAfterShapeAdjustment,'-sr');hold on; plot(lambdas,'-sb');
title('Lambdas  ==> Red : after shape adjustment');


% Reprojection Error
inEr = computeReprojectionError(meanshapeTranslatedRotatedScaled,K,keypoints(:,1:2)');
poseEr = computeReprojectionError(W_p',K,keypoints(:,1:2)');
shEr = computeReprojectionError(W_sh',K,keypoints(:,1:2)');

figure;
plot(inEr,'-sr');hold on; plot(poseEr,'-sb'); plot(shEr,'-sg');
plot(mean(inEr)*ones(size(inEr,2)),'r');plot(mean(poseEr)*ones(size(poseEr,2)),'b');plot(mean(shEr)*ones(size(shEr,2)),'g');
title('Reprojection Error ==> red : initial , blue : after Pose Ad, green : after Shape Ad');

%Rotation and translation Error
pose = importdata('../../ceres_output_singleViewPoseAdjuster.txt' );
translationError = pose(10:end) - X3d_gt
% R1= (reshape(pose(1:9),3,3))*roty(az)*rotx(pitch_gt);
% R2 = roty(az_gt)*rotx(pitch_gt);
% rotationError = 180*( rotm2eul( R1 ) - rotm2eul( R2 ) )/pi




