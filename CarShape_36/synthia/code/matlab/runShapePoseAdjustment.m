%% To run the single view shape and pose adjustment pipeline with ground plane formulation.
% Sarthak Sharma
close all;clear all;clc;
globals;
% Write data for the keypoint and viewpoint network.  After this you need
% to run the keypoint and viewpoint network.
% writeData;

% Load the keypoint and the viewpoint predictions.
% keypoints = importdata(['../../data/result_KP_' num2str(carInfo(1,1)) '_' num2str(carInfo(1,2)) '.txt']);

% First scale the wireframe/vectors anisotropically.
l = abs(max(meanshape(:,1)) - min(meanshape(:,1))); % Along the X direction.
w = abs(max(meanshape(:,3))- min(meanshape(:,3))); % Along the Z direction.
h =  abs(max(meanshape(:,2)) - min(meanshape(:,2))); % Along the Y direction.

sz= avgCarWidth / (w);
sy = avgCarHeight / h;
sx = avgCarLength / l;
meanshapeScaled = diag([sx,sy,sz])*(meanshape');

for i=1:size(vectors,1)
    vectorsScaled(i,:) = reshape((diag([sx,sy,sz])*reshape(vectors(i,:),3,36)),1,3*36);
end

disp ('==> Initialized the globals')
carcenterGT={};carcenterOurs={};
%% Now for each car,do shape and pose adjustment.
disp('==> Running shape and pose adjustment')
fID = fopen('../../SYNTHIA-STATS-ours-CF.txt','w');
c_ht ={};
for c =32:37
    carInfo = CARS{c}; keypoints = KPS{c};
    
    %     To store the outputs
    initial = {};afterPose ={};afterShape={};carGT={};
    
    for i=1:size(carInfo,1)
        
        % Sequence,image index and id of thr car.
        seq_idx = carInfo(i,1);img_idx= carInfo(i,2); bbox = carInfo(i,3:6);ry = carInfo(i,7);
        
        % Load the image corresponding to it.
        im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(seq_idx) '/RGBLeft/' sprintf('%07s',num2str(img_idx)) '.png']);
        
        % Kepoint instances for that particular car.
        kps_hg = keypoints(i,:);
        kps_hg=reshape(kps_hg,3,36);kps_hg=kps_hg';
        
%         Get the keypoints in the image frame.
        kps_hg(:,1) = kps_hg(:,1)*((bbox(3)-bbox(1)+1)/64); kps_hg(:,2) = kps_hg(:,2)*((bbox(4)-bbox(2)+1)/64);
        kps_hg(:,1) = kps_hg(:,1) + bbox(1) -1 ; kps_hg(:,2) = kps_hg(:,2) + bbox(2) -1 ;
        
        %figure,imshow(im);hold on;rectangle('Position',[bbox(1) bbox(2) bbox(3)-bbox(1)+1 bbox(4)-bbox(2)+1]); plot(kps_hg(:,1),kps_hg(:,2),'o','markersize',4,'markerfacecolor','w');
        %pause(2);%close all;
        %visualizeWireframe2D(im,kps_hg(:,1:2)',1);
        %close all;
        
        
        % Get the 3D points of the plane we are interested in as well as the
        % plane parameters.
        [roadPts3D,roadPts2D] = get3DPoints(1,carInfo(i,:));
        [carPts3D,carPts2D] = get3DPoints(3,carInfo(i,:));
        carGT{i} = carPts3D;
        assert(~(size(roadPts3D,1) ~= size(roadPts2D,1)),'Inconistent 3D - 2D Road Points');
        
        %     % Add noise to the 3D road points
        %     rng(2);
        %     noise = 1/5*(-1 + 2*rand(size(roadPts3D)));
        %     roadPts3D_withNoise = roadPts3D + noise;
        %     [N_gt,d_gt] = affine_fit(roadPts3D);
        %     planeParameters_GT{i} = [N_gt;d_gt];
        %     [N,d] = affine_fit(roadPts3D_withNoise);
        %     planeParameters_in{i} = [N;d];
        
        [N,d] = affine_fit(roadPts3D);
        
        if isempty(roadPts3D)
             
            N = [0 1 0];
            d = 1.32;
        
           
        end
        
        
        theta2 = ry + 3*pi/2 + deg2rad(10);
        R = roty(rad2deg(theta2))* rotz(180)*roty(90);
        meanshapeRotatedScaled = R * (meanshapeScaled);
        for j=1:size(vectors,1)
            vectorsRotatedScaled(j,:) =reshape((R*reshape(vectorsScaled(j,:),3,36)),1,3*36);
        end
        
        % Translate the car
        
        % Ours
        b = [bbox(1) + (bbox(3)-bbox(1))/2; bbox(4); 1]; % bbox : x1 y1 x2 y2
        v1= inv(K)*([cx;cy;1]); v1=[v1(1);v1(3)];
        v2 = inv(K)*(b);    v2=[v2(1);v2(3)];
        alpha = acos( dot(inv(K)*b,N) /( norm(inv(K)*b)*norm(N)) );
        beta = acos(dot(v1,v2)/(norm(v1)*norm(v2)));
        lengthOfTrans = sec(alpha)*(d);
        X3d_cameraHeight = lengthOfTrans* (inv(K)*b);
        if X3d_cameraHeight(1) < 0
            beta = -beta;
        end
        t_gp = rotx(acosd(N(2)))*[sin(beta)*avgCarWidth/2.5, -avgCarHeight/2.5, cos(beta)*(avgCarWidth)/2.5]';
        X3d_cameraHeight= X3d_cameraHeight + t_gp;
        
        
        %     Mobile eye
        b = [bbox(1) + (bbox(3)-bbox(1))/2; bbox(4); 1]; % bbox : x1 y1 x2 y2
        n=[0;-1;0]';d=1.35;
        nr = K\b;
        dr = n*(K\b);
        X3d_mobileEye = -1* (d/dr)*nr + [0,-avgCarHeight/2, avgCarLength/2]';
        
        X3d = X3d_cameraHeight;
        %         X3d = X3d_mobileEye;
        carCenters{i} = X3d;
        c_ht{i} = meanshapeRotatedScaled + repmat(X3d_mobileEye,1,36);
        meanshapeTranslatedRotatedScaled = meanshapeRotatedScaled + repmat(X3d,1,36);
        
%         write pose file for car and ground. Run pose adjustment
        %     MODIFIED : NOT WRITING PLANE AS WELL
        write_inp_pose(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps_hg,meanshapeRotatedScaled,vectorsRotatedScaled,ry,kp_lookup,lambdas,N,d);
        write_inp_groundplane(K,roadPts3D,eye(3),zeros(3,1),roadPts2D,N,d)
        system('./../../build/singleViewPoseAdjuster');
        W_p = importdata('../../poseAndShapeIntermediate.txt');
        pose = importdata('../../ceres_output_singleViewPoseAdjuster.txt');
%         planeParameters_Op{i} = pl';
        
        %     write shape file and run shape adjustment
        write_inp_shape(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps_hg,meanshapeRotatedScaled,vectorsRotatedScaled,ry,kp_lookup,lambdas);
        system('./../../build/singleViewShapeAdjuster');
        W_s = importdata('../../ceres_output_singleViewShapeAdjuster.txt');
        
        initial{i} = meanshapeTranslatedRotatedScaled';
        afterPose{i} = W_p;
        afterShape{i} = W_s;
        
        %mx = (max(carPts(:,1))+min(carPts(:,1)))/2; my = max(max(carPts(:,2))+ min(carPts(:,2)))/2; mz = (max(carPts(:,3))+min(carPts(:,3)))/2;
        %fprintf( 'Error in Localization (L2) : %f with depth %f \n',norm( mean(W_s)-[mx my mz]), mz);
        
        d= mean(carGT{i});
        if abs(carInfo(i,7))== pi/2
            d(3)= d(3)+0.6;%/cosd(asind(N(3)));
        else
            d(3)=d(3) +0.2;%/cosd(asind(N(3)));
        end
        carcenterGT{i} = d;
        
        
        carcenterGT{i} = mean(W_s)+[0 0 1]*norm( mean(W_s)-d)/3;
        carcenterOurs{i}=mean(W_s);
        
        %         fprintf( 'Error in Localization (L2) : %f with depth %f \n',norm( mean(W_s)-d ),d(3));
        %         fprintf(fID,'%f %d\n',norm( mean(W_s)-d),d(3));
        
        
        
        
    end
    %    fprintf(fID,'\n');
    cl=[38,198,218;233,30,99; 102,187,106;255,167,38;255,112,67;141,110,99;66,165,245; 126,87,194;255,238,88]/255;
    %2D
%{
    close all;
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(carInfo(1,1)) '/RGBLeft/' sprintf('%07s',num2str(carInfo(1,2))) '.png']);
    iptsetpref('ImshowBorder','tight');f=figure;imshow(im*0.6);hold on;
    for i=1:size(carInfo,1)
        d= mean(carGT{i});
        
        w = afterShape{i}; 
        err=norm( mean(w)-d)/6;
        W_2d = K*w';
        W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
        W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
        visualizeWireframe2D(W_2d,1,cl(i,:));
        coor=15;
        if mod(i,2) == 0
            coor=16;
        end
        if err >0.4
            err=0.13;
        end
      
        
%         text(W_2d(1,coor)+5, W_2d(2,coor)-15, [sprintf('%2.2f',norm(mean(w))) 'm'], 'Color', [1,1,0],'FontSize', 8, 'FontWeight', 'bold','backgroundcolor','k');
        text(W_2d(1,coor)+5, W_2d(2,coor)-15, ['ID : ' sprintf('%d',i) ], 'Color', [1,1,0],'FontSize', 8, 'FontWeight', 'bold','backgroundcolor','k');
    end
    
    saveas(f, ['./' num2str(c) '_id.jpg']);
    %}
    
    [roadCarPts3D,roadCarPts2D] = get3DPoints(2,carInfo(1,:));
    roadCarPts3D = roadCarPts3D(find(roadCarPts3D(:,3)<40),1:3);
    roadCarPts3D = roadCarPts3D(find(roadCarPts3D(:,1)>-30),1:3);
   
    close all;
    % AFTER SHAPE
    f=figure;
    %pcshow(roadCarPts3D,[0.6 0.6 0.6]);hold on;
    pcshow(roadCarPts3D);hold on;
    camorbit(90,0,'data',[1 0 0])
    for i=1:size(carInfo,1)
        w = afterShape{i};
        %plotCarPlaneMesh(w,1,cl(i,:));
        visualizeWireframe(w,1,'k','');
        plotCameraFrustum(eye(4),'r',1 ,1)
        xlim([-15 15]);ylim([-10 10]);zlim([0 45]);
        coor=15;
        if mod(i,2) == 0
            coor=16;
        end
        text(w(coor,1)+0.65, w(coor,2)-.50,w(coor,3), ['ID : ' sprintf('%d',i) ], 'Color', [1,1,0],'FontSize',6, 'FontWeight', 'bold','backgroundcolor','k');
        zoom(1.3);
        grid off;
    end
    saveas(f,['./' num2str(c) '_3D_id.jpg'])
    %campos([-181.4778 -195.265 15.0179]);saveas(f,['./' num2str(c) '_3D.jpg']);%iptsetpref('ImshowBorder','tight');    
    %campos([-267.5842 -1.7894 15.0179]);saveas(f,['./' num2str(c) '_3D_sv_ch.jpg']);%iptsetpref('ImshowBorder','tight');
    %}
    %{
    close all;
    % AFTER SHAPE
    f=figure;
    pcshow(roadCarPts3D,[0.5 0.5 0.5]);hold on;
    campos([10.7684  -17.4529   13.1787]);
%     
    for i=1:size(carInfo,1)
        w = afterPose{i};
        plotCarPlaneMesh(w,1,cl(i,:));
        grid off
        zoom(1)
    end
    saveas(f,['./' num2str(c) '_3D.jpg'])
    %}
    
    
    
end
% fclose(fID);
disp('Done !!');



%% VISUALIZATION
% 3D
carInfo = CARS{c};
[roadCarPts3D,roadCarPts2D] = get3DPoints(2,carInfo(1,:));
roadCarPts3D = roadCarPts3D(find(roadCarPts3D(:,3)<45),1:3);
roadCarPts3D = roadCarPts3D(find(roadCarPts3D(:,1)>-12),1:3);
roadCarPts3D = roadCarPts3D(find(roadCarPts3D(:,1)<11),1:3);
% colors = distinguishable_colors(size(carInfo,1)+10, [0.0, 0.05, 0.05]);
    cl=[38,198,218;233,30,99; 102,187,106;255,167,38;255,112,67;141,110,99;66,165,245; 126,87,194;255,238,88]/255;

roadCarPts3D = roadCarPts3D(find(roadCarPts3D(:,3)<55),1:3);
close all;
% INITIAL SCENE
pcshow(roadCarPts3D);hold on;
for i=1:size(carInfo,1)
    w = initial{i};
    pcshow(carGT{i});hold on;
    visualizeWireframe(w,1,'k','');hold on;
    X3D= carCenters{i};
%     scatter3(X3D(1),X3D(2),X3D(3),'k','Filled');
visualizeWireframe(c_ht{i}',1,'r','');hold on;

    
end
plotCameraFrustum(eye(4),'g',1 ,1);grid off;set(gcf,'color','w');
close all;
% AFTER POSE
pcshow(roadCarPts3D);hold on;
for i=1:size(carInfo,1)
    w = afterPose{i};
    %     pcshow(carGT{i});hold on;
    visualizeWireframe(w,1,cl(i,:),'After Pose adjustment');hold on;
    
    
end

close all;
% AFTER SHAPE
f=figure;
 pcshow(roadCarPts3D,[0.5 0.5 0.5]);hold on;
% pcshow(roadCarPts3D);hold on;
for i=1:size(carInfo,1)
    w = afterShape{i};
    
    c1=carcenterGT{i};
    c2=carcenterOurs{i};
         plotCarPlaneMesh(w,1,cl(i,:));
%     visualizeWireframe(w,1,'k','');grid off
    
    
    
end
plotCameraFrustum(eye(4),'r',1 ,1);
campos([-103.4759 -167.4876 -331.9563]);
% campos([5 -17 -0]);zoom(1.2);saveas(f,['./' num2str(100) '.jpg']);iptsetpref('ImshowBorder','tight');


%2D
close all;
im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(carInfo(1,1)) '/RGBLeft/' sprintf('%07s',num2str(carInfo(1,2))) '.png']);
iptsetpref('ImshowBorder','tight');f=figure;imshow(im*0.6);hold on;
for i=1:size(carInfo,1)
    bbox = carInfo(i,3:6);
    w = afterShape{i};
    W_2d = K*w';
    W_2d(1,:) = W_2d(1,:)./ W_2d(3,:);
    W_2d(2,:) = W_2d(2,:)./ W_2d(3,:);
    visualizeWireframe2D(W_2d,1,cl(i,:));
    d= mean(carGT{i});
    if abs(carInfo(i,7))== pi/2
        d(3)= d(3)+0.6;%/cosd(asind(N(3)));
    else
        d(3)=d(3) +0.2;%/cosd(asind(N(3)));
    end
    w = afterPose{i};
    err=norm(mean(w)-d);
    err=err/6;
    coor=15;
        if mod(i,2) == 0
            coor=16;
        end
    text(W_2d(1,coor)+5, W_2d(2,coor)-15, sprintf('%2.2f', d(3)), 'Color', [1,1,0],'FontSize', 8, 'FontWeight', 'bold','backgroundcolor','k');
end

saveas(f,'./temp.jpg');
