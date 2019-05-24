%% Declare the globals and initialize the data.
close all;clc;clear all;
disp('=====> Initializing data');
globals;
%% For each view,do single view shape and pose adjustment.
for view = 1:numViews
    currentCar = carInfo(view,:);
    theta2 = currentCar(5) + 3*pi/2 + 5*pi/180;
    % apply rotations to shape and vectors.
    Ry = [cos(theta2) 0 sin(theta2) ;0 1 0 ;-sin(theta2) 0 cos(theta2) ];
    R=Ry*Rx;
    singleViewShapebeforePnP = R*SHAPE{view} + repmat(CENTER{view},1,14);
    vectorsbeforePnP = VECTORS{view};
    for i=1:5
        vectorsbeforePnP(i,:) =reshape((R*reshape(vectorsbeforePnP(i,:),3,14)),1,42);
        
    end
    SHAPEbeforesingleView{view} = singleViewShapebeforePnP';
    
    % Pose adjustment.
    write_inp_pose_singleView(CENTER{view},avgCarHeight,avgCarWidth,avgCarLength,K,KPS{view},singleViewShapebeforePnP,vectorsbeforePnP);
    system('./../../build/singleViewPoseAdjuster');
    % shape adjustment.
    pose_singleView_op = importdata('./ceres_output_singleViewPoseAdjuster.txt');POSEafterPnP{view} = pose_singleView_op;
    write_inp_shape_singleView(CENTER{view},avgCarHeight,avgCarWidth,avgCarLength,K,KPS{view},singleViewShapebeforePnP,vectorsbeforePnP,pose_singleView_op);
    system('./../../build/singleViewShapeAdjuster');
    
    
    shape_singleView_op = importdata('./ceres_output_singleViewShapeAdjuster_shape.txt');
    SHAPEaftersingleView{view} =  shape_singleView_op;
    
    lambdas_singleViewshape_op = importdata('./ceres_output_singleViewShapeAdjuster_lambdas.txt');
    LaftersingleView{view} = lambdas_singleViewshape_op;
    
%     W_2d = K*(shape_singleView_op');
%     W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
%     W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
%     im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',view+start_frame-1),'.png']);
%     visualizeWireframe2D(im,W_2d);
%     pause(2);
%     close all;
    
end
%% Now for each view,we have single view pose and shape adjusted. Now run multiview adjuster that reduces the lambda reprojection error enforcing translation,rotation and lambda regularization.
write_inpFile_multiviewadjuster();
disp('====> Running shape adjustment')
system('./../../build/multiViewShapeandPoseAdjuster');
%%
shape = importdata('./ceres_output_mutliViewShapeAdjuster.txt');
close all;

for i=1:15
    % first do R*(X) + t to get in the correct orientation.
    W = shape((i-1)*14 +1 : (i-1)*14 +1 +14-1,: );
    % now K*(...);
    W_2d = K*(W');
    % now pi(...);
    W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
    W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
    
    
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',start_frame + i-1),'.png']);
    %figure,imshow(im);hold on;rectangle('Position',[currentCar(1),currentCar(2),currentCar(3)-currentCar(1)+1,currentCar(4)-currentCar(2)+1]);scatter(currKps(:,1),currKps(:,2),'red');
    visualizeWireframe2D(im,W_2d);
    pause(5)
%     visualizeWireframe2DandSaveImage(im,W_2d,i);
    
    close all;
    
    
    
end




%}