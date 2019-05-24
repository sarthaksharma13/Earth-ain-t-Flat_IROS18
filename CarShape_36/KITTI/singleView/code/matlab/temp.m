%% To run the single view shape and pose adjustment pipeline with ground plane formulation.
% Sarthak Sharma

clc; clear all; close all;
%%
% Declare/ read the global variables
% Camera calibratin matrix
K = [721.537700 0.000000 609.559300; 0.000000 721.537700 172.854000; 0 0 1];

% Load the meanshape and the basis vectors.
meanshape = importdata('../../data/meanShape_p.txt');
vectors = importdata('../../data/vectors_p.txt');

% Average H, W ,L of the car in metres.
avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Keypoint lookup table for binary weight.
kp_lookup= load('../../data/kpLookup_azimuth.mat');kp_lookup = kp_lookup.kpLookup;

% Load the keypoint and the viewpoint predictions.
keypoints = importdata('../../data/result_KP.txt');
az = .5;

% First scale the wireframe/vectors anisotropically.
l = abs(max(meanshape(:,1)) - min(meanshape(:,1))); % Along the X direction.
w = abs(max(meanshape(:,3))- min(meanshape(:,3))); % Along the Z direction.
h =  abs(max(meanshape(:,2)) - min(meanshape(:,2))); % Along the Y direction.

sx= avgCarWidth / w;
sz = avgCarHeight / h;
sy = avgCarLength / l;
W=meanshape;
meanshapeScaled = diag([sx,sy,sz])*(meanshape');


for i=1:size(vectors,1)
    vectorsScaled(i,:) = reshape((diag([sx,sy,sz])*reshape(vectors(i,:),3,36)),1,3*36);
end

% Apply the transformations to bring from shape net object frame to KITTI
% frame.
R = rotz(180)*roty(90); % Bring from shape net object frame to KITTI frame.
R = roty(az)*R; % Apply azimuth

meanshapeRotatedScaled = R * (meanshapeScaled);
for j=1:size(vectors,1)
    vectorsRotatedScaled(j,:) =reshape((R*reshape(vectorsScaled(j,:),3,36)),1,3*36);
end

% Translate the car
bboxANDplane = importdata('../../data/bboxANDPlane.txt');
bbox = bboxANDplane(1,:); % x y w h
plane = bboxANDplane(2,:);
b = [bbox(1) + (bbox(3)-1)/2; bbox(2)+bbox(4)-1; 1];
n = -1*plane(1,1:3); d = plane(1,4);
alpha = pi - acos(dot(inv(K)*b,n))
X3d = d*sec(alpha) * (inv(K)*b);

t_gp = (avgCarLength/2)*[0;0;1] + (avgCarHeight/2) - dot((avgCarLength/2)*[0;0;1] + (avgCarHeight/2)*[0;-1;0],n); % Translate to the center of the car along the road plane

X3d= X3d + t_gp;
meanshapeTranslatedRotatedScaled = meanshapeRotatedScaled + repmat(X3d,1,36);

% FOR VISUALIZATION OF THE INITIAL WIREFRAME
w2d= K*meanshapeTranslatedRotatedScaled;
w2d(1,:) = w2d(1,:)./w2d(3,:);
w2d(2,:) = w2d(2,:)./w2d(3,:);
% visualizeWireframe2D(im,w2d);
visualizeWireframe3D(meanshapeTranslatedRotatedScaled);

%%
% write pose file
write_inp_pose_syn(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,keypoints,meanshapeTranslatedRotatedScaled,vectorsRotatedScaled,az,kp_lookup,-1*n,d);
system('./../../build/poseAdjuster');

%%
% write the ground plane file

%%
% write the shape file
write_inp_shape_syn(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,keypoints,meanshapeTranslatedRotatedScaled,vectorsRotatedScaled,az,kp_lookup);
system('./../../build/shapeAdjuster');
W = importdata('../../ceres_output_singleViewShapeAdjuster.txt');
visualizeWireframe3D(W');
% W_2d = K*W';
% W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
% W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);

% Visualize the shaep and pose adjusted wireframe in 2D
% visualizeWireframe2D(im,W_2d);