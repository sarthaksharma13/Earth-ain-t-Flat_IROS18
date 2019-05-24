%% To run the single view shape and pose adjustment pipeline with ground plane formulation.
% Sarthak Sharma
close all;clear all;clc;
globals;
disp('==> Initialized globals')
%% Now for each car,do shape and pose adjustment.
disp('==> Running shape and pose adjustment')
write_inp_pose(CENTER,avgCarHeight,avgCarWidth,avgCarLength,K,KPS,WF,VEC,RY,kp_lookup,lambdas,N,d)
system('./../../build/singleViewPoseAdjuster');
% output after pose adjustment
poseOp = importdata('../../ceres_output_singleViewPoseAdjuster.txt');
shapeAfterPose = importdata('../../poseAndShapeIntermediate.txt');
SHAPE ={};% final shape
for i=1:size(carInfo,1)
    
    pose = poseOp((i-1)*12 + 1:i*12,:);
    write_inp_shape(CENTER{i},avgCarHeight,avgCarWidth,avgCarLength,K,KPS{i},WF{i},VEC{i},RY{i},kp_lookup,lambdas,pose);
    system('./../../build/singleViewShapeAdjuster');
    SHAPE{i}=importdata('../../ceres_output_singleViewShapeAdjuster.txt');
    d= mean(carGT{i});
    if abs(carInfo(i,7))== pi/2
        d(3)= d(3)+0.6;%/cosd(asind(N(3)));
    else
        d(3)=d(3) +0.2;%/cosd(asind(N(3)));
    end
    fprintf( 'Error in Localization (L2) : %f with depth %f \n',norm( mean(SHAPE{i})-d ),d(3));
    
end
disp('Done !!');
%% VISUALIZATION
% 3D
carInfo = CARS{c};
[roadCarPts3D,roadCarPts2D] = get3DPoints(2,carInfo(1,:));
colors = distinguishable_colors(size(carInfo,1), [0.0, 0.05, 0.05]);
roadCarPts3D = roadCarPts3D(find(roadCarPts3D(:,3)<65),1:3);
close all;
% INITIAL SCENE
pcshow(roadCarPts3D);hold on;
for i=1:size(carInfo,1)
    w = initial{i};
    pcshow(carGT{i});hold on;
    visualizeWireframe(w,1,colors(i,:),'Initial Scene');hold on;
    X3D= carCenters{i};
    scatter3(X3D(1),X3D(2),X3D(3),'k','Filled');
    
end

close all;
% AFTER POSE
pcshow(roadCarPts3D);hold on;
for i=1:size(carInfo,1)
    w = afterPose{i};
    %     pcshow(carGT{i});hold on;
    visualizeWireframe(w,1,colors(i,:),'After Pose adjustment');hold on;
    
    
end

close all;
% AFTER SHAPE
pcshow(roadCarPts3D);hold on;
for i=1:size(carInfo,1)
    w = afterShape{i};
    %     pcshow(carGT{i});hold on;
    visualizeWireframe(w,1,colors(i,:), 'After Shape Adjustment');hold on;
    
end

%2D
close all;
im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(carInfo(1,1)) '/RGBLeft/' sprintf('%07s',num2str(carInfo(1,2))) '.png']);
figure;imshow(im*1.3);hold on; kps_hg = KPS{2};plot(kps_hg(:,1),kps_hg(:,2),'o','markersize',4,'markerfacecolor','w');hold on;
for i=2
    bbox = carInfo(i,3:6);
    w = SHAPE{i};
    W_2d = K*w';
    W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
    W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
    visualizeWireframe2D(W_2d,1,'r');
    w=WF{i} + repmat(CENTER{i},1,36);
    W_2d = K*w;
    W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
    W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
    %     visualizeWireframe2D(W_2d,1,'b');
end
