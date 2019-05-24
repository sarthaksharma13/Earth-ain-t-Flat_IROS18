seq_idx=2;
start_frame=95;
end_frame = 100;
carID =1;
groundPlaneNormal=[0;-1;0];
H = 1.72;
numViews = end_frame - start_frame +1;

% Get the car on its wheels.
theta1 = pi/2;
Rx= [ 1 0 0;0 cos(theta1) -sin(pi/2); 0 sin(theta1) cos(theta1)];

% write data for the hourglasas as well as load all the object info in
% carInfo.
writeData();

% camera calibration matrix
K = [721.500000 0.000000 609.500000; 0.000000 721.500000 172.800000; 0.000000 0.000000 1.000000];

% mean shape and vectors
meanshape = importdata('../../data/meanShape.txt');
vectors = importdata('../../data/vectors.txt');

% avg L ,W, H
avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Kps lookUp table and the predicted keypoints
kp_lookup= load('../../data/kpLookup_azimuth.mat');kp_lookup = kp_lookup.kpLookup;
kps_hg = importdata('../../data/keypoints_2_95-100_1.txt'); % numViewsx42;


% Get the scaling factors.
l = abs(max(meanshape(:,2)) - min(meanshape(:,2))); % Along the Y direction.
w = abs(max(meanshape(:,1))- min(meanshape(:,1))); % Along the X direction.
h =  abs(max(meanshape(:,3)) - min(meanshape(:,3))); % Along the Z direction.

sx= avgCarWidth / w;
sz = avgCarHeight / h;
sy = avgCarLength / l;

%}
