% The cars for which we want to show the results : KITTI seq, KITTI image index(0 based) , KITTI ID.
cars = [2 98 1;10 1 0; 4 197 20;8 126 12;2 90 1;9 42 1];
% Camera calibratin matrix
K = [721.500000 0.000000 609.500000; 0.000000 721.500000 172.800000; 0.000000 0.000000 1.000000];

% Load the meanshape and the basis vectors.
meanshape = importdata('../../data/meanShape.txt');
% left = meanshape(1:18,:);
% right = meanshape(19:36,:);
% left(18:1,:)=left(18:-1:1,:);
% right(18:1,:)=right(18:-1:1,:);
vectors = importdata('../../data/vectors.txt');

% Average H, W ,L of the car in metres.
avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Keypoint lookup table for binary weight.
kp_lookup= load('../../data/keypoint_visibility_36.mat');kp_lookup = kp_lookup.keypoint_visibility_36;

% Image and tracklet info
cam =2 ;
root_dir = '/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT';
label_dir = fullfile([root_dir '/KITTITracking/'], sprintf('label_%02d/',cam));

% Information of each car : bbox,azimuth,translation vector
carInfo=[];

for i=1:size(cars,1)
    seq_idx = cars(i,1);img_idx= cars(i,2); carID = cars(i,3);
    tracklets = readLabels(label_dir, seq_idx);
    t_frm = tracklets{img_idx+1};
    CARS = bbox_ry(t_frm,carID); 
    carInfo = [carInfo;CARS{1}'];
    
end
lambdas=10*[0.0208364480250510,0.00967136192730910,0.00717094319071054,0.00565548391744564,0.00467453361084465,0.00329643528181894,0.00209592766309844,0.00163342705234948,0.00104417262030107,0.000949492925761423,0.000805245112797721,0.000794819762941728,0.000738459491936479,0.000637465648951683,0.000545710387292436,0.000530415743589159,0.000430982248425289,0.000383471123360310,0.000364060566388879,0.000335845449738752,0.000293254266498406,0.000280809402233219,0.000253979011166301,0.000250439803923838,0.000239639422099641,0.000227739460767099,0.000212634691872905,0.000201197758665930,0.000191438687798717,0.000158787716884223,0.000153031995700756,0.000137391994149763,0.000122187478151364,0.000116773767656378,0.000105296101573683,9.79367791848786e-05,9.60947836203073e-05,8.04996641937673e-05,7.61223597074321e-05,6.60262537278197e-05,5.68724957185047e-05,5.44969447067057e-05];















%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OLD CODE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%



% seq_idx=2;img_idx =98;carID =1;
% mean shape and vectors
% meanshape = importdata('../../data/meanShape_ShapeNet.txt');
% vectors = importdata('../../data/vectors_ShapeNet.txt');

% % avg L ,W, H
% avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Kps weight from HG and LookUp table

% kps_hg = importdata('../../data/kps_2_98_1.txt');
% kps_hg = importdata('../../data/kps_10_1_0.txt');
% kps_hg = importdata('../../data/kps_3_41_1.txt');
% kps_hg = importdata('../../data/kps_4_197_20.txt');
% kps_hg = importdata('../../data/kps_8_126_12.txt');
% kps_hg = importdata('../../data/kps_2_90_1.txt');


% kps_hg=reshape(kps_hg,3,14);kps_hg=kps_hg';
%  kp_lookup= load('../../data/kpLookup_azimuth.mat');kp_lookup = kp_lookup.kpLookup;


% % Image and tracklet info
% cam =2 ;
% root_dir = '/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT';
% label_dir = fullfile([root_dir '/KITTITracking/'], sprintf('label_%02d/',cam));
% image_dir = fullfile([root_dir '/KITTITracking/dataset_left/training/' sprintf('image_%02d/%02d/image_0/',cam, seq_idx)] );
% im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',img_idx),'.png']);
% car_Info=[];
% for i=1:size(cars,1)
% tracklets = readLabels(label_dir, seq_idx);
% t_frm = tracklets{img_idx+1};
% CARS = bbox_ry(t_frm,carID); % functions returns all the tracklets of type car , each tracklet x1 y1 x2 y2 ry
% % car= CARS{1};
% car_Info = [CARS{1}
%     
% end
