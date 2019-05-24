close all;clear all;clc;

globals;
% Write data for the keypoint and viewpoint network.  After this you need
% to run the keypoint and viewpoint network.
% writeData;

% Load the keypoint and the viewpoint predictions.
keypoints = importdata('../../data/result_KP.txt');
% viewpoint = importdata('../../data/result_VP_k.txt');


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

disp ('==> Initialized the globals')
%% Now for each car,do shape and pose adjustment.
for i=1:size(carInfo,1)
    
    % Sequence,image index and id of thr car.
    seq_idx = cars(i,1);img_idx= cars(i,2); id = cars(i,3);
    
    % Load the image corresponding to it.
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',img_idx),'.png']);
    
    % Kepoint instances for that particular car.
    kps_hg = keypoints(i,:);
    kps_hg=reshape(kps_hg,3,36);kps_hg=kps_hg';
    
    % Get the keypoints in the image frame.
    kps_hg(:,1) = kps_hg(:,1)*((carInfo(i,3)-carInfo(i,1)+1)/64); kps_hg(:,2) = kps_hg(:,2)*((carInfo(i,4)-carInfo(i,2)+1)/64);
    kps_hg(:,1) = kps_hg(:,1) + carInfo(i,1) -1 ; kps_hg(:,2) = kps_hg(:,2) + carInfo(i,2) -1 ;
    figure,imshow(im);hold on;rectangle('Position',[carInfo(i,1),carInfo(i,2),carInfo(i,3)-carInfo(i,1)+1,carInfo(i,4)-carInfo(i,2)+1]);scatter(kps_hg(:,1),kps_hg(:,2),'red');
    pause(3)
    
    ry = carInfo(i,5);
    theta2 = ry + 3*pi/2 + deg2rad(15);
    R = roty(rad2deg(theta2))* rotz(180)*roty(90);
    
        
    meanshapeRotatedScaled = R * (meanshapeScaled);
    for j=1:size(vectors,1)
        vectorsRotatedScaled(j,:) =reshape((R*reshape(vectorsScaled(j,:),3,36)),1,3*36);
    end
    
    
    % Get the 3D coordinate of the car and translate the mean shape there.
    n=[0;-1;0];
    H = 1.65;
    x2d= [(carInfo(i,1) + carInfo(i,3))/2; carInfo(i,4);1];
    nr = K\x2d;
    dr = n'*(K\x2d);
    X3d = -1* (H/dr)*nr;
    X3d= X3d + [0; -avgCarHeight/2; avgCarLength/2];
    meanshapeTranslatedRotatedScaled = meanshapeRotatedScaled + repmat(X3d,1,36);
    
    % FOR VISUALIZATION OF THE INITIAL WIREFRAME
    w2d= K*meanshapeTranslatedRotatedScaled;
    w2d(1,:) = w2d(1,:)./w2d(3,:);
    w2d(2,:) = w2d(2,:)./w2d(3,:);
    visualizeWireframe2D(im,w2d);
    
    pause(3)
    
    
    % Run the shape and pose optimization.
   
    % write pose file
    write_inp_pose(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps_hg,meanshapeTranslatedRotatedScaled,vectorsRotatedScaled,ry,kp_lookup,lambdas);
    system('./../../build/singleViewPoseAdjuster');
    op_pose = importdata('../../ceres_output_singleViewPoseAdjuster.txt');
    deltaT = op_pose(end-3 +1:end,:);
    finalTrans = X3d + deltaT;
    W = importdata('../../poseAndShapeIntermediate.txt');
    W_2d = K*W';
    W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
    W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
    visualizeWireframe2D(im,W_2d);
    pause(3);
    
    
    write_inp_shape(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps_hg,meanshapeTranslatedRotatedScaled,vectorsRotatedScaled,ry,kp_lookup,lambdas);
    system('./../../build/singleViewShapeAdjuster');
    W = importdata('../../ceres_output_singleViewShapeAdjuster.txt');
    W_2d = K*W';
    W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
    W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
    % Visualize the shaep and pose adjusted wireframe in 2D
    visualizeWireframe2D(im,W_2d);
    pause(3);
  
    clc;
    disp(' Error in translation : ');
    finalTrans- (carInfo(i,end-3+1:end)' + [0; -avgCarHeight/2; 0] )
    
    
    
end
















%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OLD CODE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%


% %first scale the wireframe/vectors anisotropically
% l = abs(max(meanshape(:,2)) - min(meanshape(:,2))); % Along the Y direction.
% w = abs(max(meanshape(:,1))- min(meanshape(:,1))); % Along the X direction.
% h =  abs(max(meanshape(:,3)) - min(meanshape(:,3))); % Along the Z direction.
%
% sx= avgCarWidth / w;
% sz = avgCarHeight / h;
% sy = avgCarLength / l;
% W=meanshape;
% meanshape = diag([sx,sy,sz])*(meanshape');
%
% for i=1:5
%     vectors(i,:) = reshape((diag([sx,sy,sz])*reshape(vectors(i,:),3,14)),1,42);
% end

% Rotations to bring in KITTI frame and also for get alligned roughly with
% the viewpoint of the car. Initially the meanshape is in PASCAL convention
% ,hence needs to be taken to KITTI Frame by adding 3pi/2; Also intitially
% the car is on its roof. So need to flip it first so that it comes on the
% wheel, and then rotate according to PASCAL -> KITTI using ry.
% theta1 = pi/2;
% theta2 = car(5) + 3*pi/2;
%
%
%
% Rx= [ 1 0 0;0 cos(theta1) -sin(pi/2); 0 sin(theta1) cos(theta1)];
%
% Ry = [cos(theta2 + normrnd(0,15*pi/180)) 0 sin(theta2 + normrnd(0,15*pi/180) ) ;0 1 0 ;-sin(theta2+normrnd(0,15*pi/180)) 0 cos(theta2+normrnd(0,15*pi/180)) ];
% R=Ry*Rx;

% R=eye(3)
% meanshape = R * (meanshape);
% for i=1:5
%     vectors(i,:) =reshape((R*reshape(vectors(i,:),3,14)),1,42);
% end
% ry = car(5) + 3*pi/2;
% ry = car(5) + normrnd(0,15*pi/180); % corrupt by noise
% R_az = [cos(ry) 0 sin(ry);0 1 0;-sin(ry) 0 cos(ry)];
% meanshape = R_az * (meanshape);
% for i=1:5
%     vectors(i,:) =reshape((R*reshape(vectors(i,:),3,14)),1,42);
% end

%
% % get the keypoints in the image frame.
% kps_hg(:,1) = kps_hg(:,1)*((car(3)-car(1)+1)/64); kps_hg(:,2) = kps_hg(:,2)*((car(4)-car(2)+1)/64);
% kps_hg(:,1) = kps_hg(:,1) + car(1) -1 ; kps_hg(:,2) = kps_hg(:,2) + car(2) -1 ;
% figure,imshow(im);hold on;rectangle('Position',[car(1),car(2),car(3)-car(1)+1,car(4)-car(2)+1]);scatter(kps_hg(:,1),kps_hg(:,2),'red');


% % Get the 3D coordinate of the car and translate the mean shape there.
% n=[0;-1;0];
% H = 1.65;
% x2d= [(car(1) + car(3))/2; car(4);1];
% nr = K\x2d;
% dr = n'*(K\x2d);
% X3d = -1* (H/dr)*nr;

% X3d = [car(6);car(7);car(8)];


% X3d= X3d + [0; -avgCarHeight/2; avgCarLength/2];
% meanshape = meanshape + repmat(X3d,1,14);
% w2d= K*meanshape;
% w2d(1,:) = w2d(1,:)./w2d(3,:);
% w2d(2,:) = w2d(2,:)./w2d(3,:);
% visualizeWireframe2D(im,w2d);
%

%% Run the shape and pose optimization.
% close all;
% % write pose file
% write_inp_pose(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps_hg,meanshape,vectors,theta2,kp_lookup);
% system('./../../build/singleViewPoseAdjuster');


% pose_op = importdata('./ceres_output_singleViewPoseAdjuster.txt');
% meanshape=W;
% meanshape = diag([sx,sy,sz])*(meanshape');
%
% for i=1:5
%     vectors(i,:) = reshape((diag([sx,sy,sz])*reshape(vectors(i,:),3,14)),1,42);
% end
%
% deltaR = reshape(pose_op(1:9,1),3,3);
% deltaT = pose_op(10:12,1);
% meanshape=deltaR*meanshape;
% R=deltaR*Ry*Rx;
%
%
% meanshape = R * (meanshape);
% for i=1:5
%     vectors(i,:) =reshape((R*reshape(vectors(i,:),3,14)),1,42);
% end
%


% write_inp_shape(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps_hg,meanshape,vectors,theta2,kp_lookup);
% system('./../../build/singleViewShapeAdjuster');
%
% W = importdata('./ceres_output_singleViewShapeAdjuster.txt');
% W_2d = K*W';
% W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
% W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
%
% visualizeWireframe2D(im,W_2d);


