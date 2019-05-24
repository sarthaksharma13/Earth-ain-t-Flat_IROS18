close all;clc;clear all;
disp('=====> Initializing data');
globals;

% Construct cells for each view.
SHAPE = {};KPS = {};CENTRE={};WEIGHT={};VECTORS={};
for view=1:numViews
    currentCar = carInfo(view,:);
    %im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',start_frame + view-1),'.png']);
    
    %First scale the mean shape and vectors by scaling factors
    SHAPE{view} = diag([sx,sy,sz])*(meanshape');
    VECTORS{view} = vectors;
    for j=1:5
        VECTORS{view}(j,:) = reshape((diag([sx,sy,sz])*reshape(VECTORS{view}(j,:),3,14)),1,42);
    end
    
    % Then rotate the meanshape and the basis vectors.
    theta2 = currentCar(5) + 3*pi/2;
    Ry = [cos(theta2 + normrnd(0,15*pi/180)) 0 sin(theta2 + normrnd(0,15*pi/180) ) ;0 1 0 ;-sin(theta2+normrnd(0,15*pi/180)) 0 cos(theta2+normrnd(0,15*pi/180)) ];
    R= Ry*Rx;
    SHAPE{view} = R*SHAPE{view};
    for j=1:5
         VECTORS{view}(j,:) = reshape((R*reshape(VECTORS{view}(j,:),3,14)),1,42);
    end
    
    % calcluate the car center
    x2d= [(currentCar(1) + currentCar(3))/2; currentCar(4);1];
    nr = K\x2d;
    dr = groundPlaneNormal'*(K\x2d);
    X3d = -1* (H/dr)*nr;
    X3d= X3d + [0; -avgCarHeight/2; avgCarLength/2];
    CENTER{view}= X3d;
    
    % Translate the shape to the center.
    SHAPE{view}= SHAPE{view} + repmat(CENTER{view},1,14);
%     w2d = K*SHAPE{view};
%     w2d(1,:) = w2d(1,:)./w2d(3,:);
%     w2d(2,:) = w2d(2,:)./w2d(3,:);
%     visualizeWireframe2D(im,w2d);
   
    
    % Keypoints and their weights .
    currKps=reshape(kps_hg(view,:),3,14);currKps=currKps';
    currKps(:,1) = currKps(:,1)*((currentCar(3)-currentCar(1)+1)/64); currKps(:,2) = currKps(:,2)*((currentCar(4)-currentCar(2)+1)/64);
    currKps(:,1) = currKps(:,1) + currentCar(1) -1 ; currKps(:,2) = currKps(:,2) + currentCar(2) -1 ;
    wV = kp_lookup(floor(theta2),:)/(sum(kp_lookup(floor(theta2),:)));
    wHG = currKps(:,3);
    currKps(:,3) = 0.7*(wV') + 0.3*wHG;
    KPS{view} = currKps;
    %figure,imshow(im);hold on;rectangle('Position',[currentCar(1),currentCar(2),currentCar(3)-currentCar(1)+1,currentCar(4)-currentCar(2)+1]);scatter(currKps(:,1),currKps(:,2),'red');
   
    
end

disp('=====> Writing input file');
writeInputFile();


%% Run the shape and pose optimization.
disp('=====> Adjusting pose and shape');
close all;
system('../../build/shapeAndPoseAdjuster');
W = importdata('./ceresOutputFile.txt');
for i=1:numViews
   
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',start_frame + i-1),'.png']);
    figure,imshow(im);hold on;rectangle('Position',[currentCar(1),currentCar(2),currentCar(3)-currentCar(1)+1,currentCar(4)-currentCar(2)+1]);scatter(currKps(:,1),currKps(:,2),'red');
    W_2d = K*W((i-1)*14+1:(i-1)*14+1+14-1,:)';
    W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
    W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
    visualizeWireframe2D(im,W_2d);
    pause(1);
 
end

close all;


