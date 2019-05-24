% The cars for which we want to show the results : Seq,imagenum, bbox, ry
% carInfo = [2 19 847 445 903 490 pi/2];
carInfo = [ 2 19 847 445 903 490 pi/2 ; 2 19 805 417 845 457 pi/2];
% carInfo =[1 106 285 442 552 574 pi/2;1 106 885 470 1105 655 -pi/2];
% carInfo = [2 62 1219 524 1697 722 pi/2];
% carInfo = [1 192 376 535 741 713 pi/2; 1 192 749 532 859 602 pi/2; 1 192 923 535 1038 626 -pi/2];
% carInfo = [1 178 1040 700 1090 750 pi/2];


% Load the meanshape and the basis vectors.
meanshape = importdata('../../data/meanShape.txt');
vectors = importdata('../../data/vectors.txt');

% Average H, W ,L of the car in metres.
avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Keypoint lookup table for binary weight.
kp_lookup= load('../../data/keypoint_visibility_36.mat');kp_lookup = kp_lookup.keypoint_visibility_36;

% Load the keypoint and the viewpoint predictions.
keypoints = importdata(['../../data/result_KP_' num2str(carInfo(1,1)) '_' num2str(carInfo(1,2)) '.txt']);

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

f=847.630211643;
fx=f;fy=f;cx=960;cy=540;
K = [fx 0 cx; 0 fy cy ; 0 0 1];

% need the following for each car
KPS={};CENTER={};WF={};VEC={};RY={};GP={};carGT={};
for i=1:size(carInfo,1)
    
    seq_idx = carInfo(i,1);img_idx= carInfo(i,2); bbox = carInfo(i,3:6);ry = carInfo(i,7);
    % Kepoint instances for that particular car.
    kps_hg = keypoints(i,:);
    kps_hg=reshape(kps_hg,3,36);kps_hg=kps_hg';
    
    % Get the keypoints in the image frame.
    kps_hg(:,1) = kps_hg(:,1)*((bbox(3)-bbox(1)+1)/64); kps_hg(:,2) = kps_hg(:,2)*((bbox(4)-bbox(2)+1)/64);
    kps_hg(:,1) = kps_hg(:,1) + bbox(1) -1 ; kps_hg(:,2) = kps_hg(:,2) + bbox(2) -1 ;
    KPS{i} = kps_hg;
    
    
    [roadPts3D,roadPts2D] = get3DPoints(1,carInfo(i,:));
    [carPts3D,carPts2D] = get3DPoints(3,carInfo(i,:));
    carGT{i} = carPts3D;
    assert(~(size(roadPts3D,1) ~= size(roadPts2D,1)),'Inconistent 3D - 2D Road Points');
    [N,d] = affine_fit(roadPts3D);
    
    theta2 = ry + 3*pi/2 + deg2rad(10);
    R = roty(rad2deg(theta2))* rotz(180)*roty(90);
    meanshapeRotatedScaled = R * (meanshapeScaled);
    WF{i}=meanshapeRotatedScaled;
    for j=1:size(vectors,1)
        vectorsRotatedScaled(j,:) =reshape((R*reshape(vectorsScaled(j,:),3,36)),1,3*36);
    end
    VEC{i} = vectorsRotatedScaled;
    
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
    
    CENTER{i}=X3d_cameraHeight;
    
    RY{i}=ry;
    GP{i}=[N d];
    
end

lambdas=[
    0.0208
    0.0097
    0.0072
    0.0057
    0.0047
    0.0033
    0.0021
    0.0016
    0.0010
    0.0009
    0.0008
    0.0008
    0.0007
    0.0006
    0.0005
    0.0005
    0.0004
    0.0004
    0.0004
    0.0003
    0.0003
    0.0003
    0.0003
    0.0003
    0.0002
    0.0002
    0.0002
    0.0002
    0.0002
    0.0002
    0.0002
    0.0001
    0.0001
    0.0001
    0.0001
    0.0001
    0.0001
    0.0001
    0.0001
    0.0001
    0.0001
    0.0001];


