% seq_idx=4; start_frame=40;end_frame = 50; carID =2;
% seq_idx=10; start_frame=0;end_frame = 50; carID =0;
% seq_idx = 5;start_frame = 190; end_frame = 230; carID = 31;
% seq_idx = 5;start_frame = 182; end_frame = 191; carID = 12;
% seq_idx = 5;start_frame = 193; end_frame = 200; carID = 17;
% seq_idx = 7;start_frame = 151;end_frame = 159;carID =14;
% seq_idx = 11;start_frame = 0;end_frame = 40;carID =0;
seq_idx = 9;start_frame = 48;end_frame = 62;carID =9;

groundPlaneNormal=[0;-1;0];
H = 1.72;
numViews = end_frame - start_frame +1;

% Get the car on its wheels.
theta1 = pi/2;
Rx= [ 1 0 0;0 cos(theta1) -sin(pi/2); 0 sin(theta1) cos(theta1)];

% Car Information.
carInfo =[];
cam =2 ;
root_dir = '/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT';
label_dir = fullfile([root_dir '/KITTITracking/'], sprintf('label_%02d/',cam));
image_dir = fullfile([root_dir '/KITTITracking/dataset_left/training/' sprintf('image_%02d/%02d/image_0/',cam, seq_idx)] );
tracklets = readLabels(label_dir, seq_idx);
for img_idx=start_frame:end_frame
    t_frm = tracklets{img_idx+1};
    car = bbox_ry(t_frm,carID);
    car=car{1};
    carInfo = [carInfo;car(1) car(2) car(3) car(4) car(5)];
end

% write data for the hourglasas as well as load all the object info in
% carInfo.
% writeData();

% camera calibration matrix
K = [721.500000 0.000000 609.500000; 0.000000 721.500000 172.800000; 0.000000 0.000000 1.000000];

% mean shape and vectors
meanshape = importdata('../../data/meanShape.txt');
vectors = importdata('../../data/vectors.txt');

% avg L ,W, H
avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Kps lookUp table and the predicted keypoints
kp_lookup= load('../../data/kpLookup_azimuth.mat');kp_lookup = kp_lookup.kpLookup;

%  kps_hg = importdata('../../data/keypoints_2_90-100_1.txt'); % numViewsx42;


% kps_hg = importdata('../../data/keypoints_4_40-55_id2.txt'); % numViewsx42;


% kps_hg = importdata('../../data/keypoints_10_0-50_0.txt'); % numViewsx42; WORKS
% WELL

% kps_hg = importdata('../../data/keypoints_5_190-230_31.txt'); % numViewx42;

%  kps_hg = importdata('../../data/keypoints_5_182-190_12.txt'); % WORKS
% WELL

% kps_hg = importdata('../../data/keypoints_5_193-200_17.txt'); % WORKS
% WELL


% kps_hg = importdata('../../data/keypoints_7_151_159.txt');% WORKS
% WELL

% kps_hg = importdata('../../data/keypoints_11_0-40_0.txt');

kps_hg = importdata('../../data/keypoints_9_48-62_9.txt');% WORKS
% WELL

% Get the scaling factors.
l = abs(max(meanshape(:,2)) - min(meanshape(:,2))); % Along the Y direction.
w = abs(max(meanshape(:,1))- min(meanshape(:,1))); % Along the X direction.
h =  abs(max(meanshape(:,3)) - min(meanshape(:,3))); % Along the Z direction.

sx= avgCarWidth / w;
sz = avgCarHeight / h;
sy = avgCarLength / l;

% Construct  cells for each view.
SHAPE = {};KPS = {};CENTRE={};VECTORS={};POSEafterPnP={};LaftersingleView={};SHAPEbeforesingleView={};SHAPEaftersingleView={};VECTORSaftersingleView={};

for view=1:numViews
    currentCar = carInfo(view,:);
   
    
    %First scale the mean shape and vectors by scaling factors
    SHAPE{view} = diag([sx,sy,sz])*(meanshape');
    VECTORS{view} = vectors;
    for j=1:5
        VECTORS{view}(j,:) = reshape((diag([sx,sy,sz])*reshape(VECTORS{view}(j,:),3,14)),1,42);
    end
    
    % calcluate the car center
    x2d= [(currentCar(1) + currentCar(3))/2; currentCar(4);1];
    nr = K\x2d;
    dr = groundPlaneNormal'*(K\x2d);
    X3d = -1* (H/dr)*nr;
    X3d= X3d + [0; -avgCarHeight/2; avgCarLength/2];
    CENTER{view}= X3d;

    theta2 = currentCar(5) + 3*pi/2;
    
    % Keypoints and their weights .
    currKps=reshape(kps_hg(view,:),3,14);currKps=currKps';
    currKps(:,1) = currKps(:,1)*((currentCar(3)-currentCar(1)+1)/64); currKps(:,2) = currKps(:,2)*((currentCar(4)-currentCar(2)+1)/64);
    currKps(:,1) = currKps(:,1) + currentCar(1) -1 ; currKps(:,2) = currKps(:,2) + currentCar(2) -1 ;
    wV = kp_lookup(floor(theta2),:)/(sum(kp_lookup(floor(theta2),:)));
    wHG = currKps(:,3);
    currKps(:,3) = 0.7*(wV') + 0.3*wHG;
    KPS{view} = currKps;
    
%     im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',start_frame + view-1),'.png']);
%     figure,imshow(im);hold on;rectangle('Position',[currentCar(1),currentCar(2),currentCar(3)-currentCar(1)+1,currentCar(4)-currentCar(2)+1]);scatter(currKps(:,1),currKps(:,2),'red','filled');
%     pause(5);
%      
    
    
    
end













%}
