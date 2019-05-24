% The cars for which we want to show the results : Seq,imagenum, bbox, ry
% carInfo = [2 19 370 512 593 618 pi/2; 2 19 715 466 790 526 pi/2; 2 19 765 519 867 590 pi/2; 2 19 847 445 903 490 pi/2; 2 19 805 417 845 457 pi/2];
% carInfo =[1 106 285 442 552 574 pi/2;1 106 885 470 1105 655 -pi/2];
% carInfo = [2 62 1219 524 1697 722 pi/2];
% carInfo = [1 192 376 535 741 713 pi/2; 1 192 749 532 859 602 pi/2; 1 192 923 535 1038 626 -pi/2];
% carInfo = [1 178 1040 700 1090 750 pi/2];


Keypoints = importdata('../../data/result_KP_2_54_90.txt');
a=load('../../data/synthia_2_54_90.mat');
% a=load('/home/sarthak/synthia_new_bbs.mat');
carInfo = [];
a=a.bbmat;
v = values(a);
k = keys(a);

for i=1:numel(keys(a))
    cars = v(i);cars=cars{1};
    im = k(i);im = im{1};
    im = strsplit(im,'_');
    seq_idx= im(1); img_idx = im(2);
    seq_idx=seq_idx{1};img_idx=img_idx{1};
    for j=1:size(cars,1)
        bbox = cars(j,1:4); ry = cars(j,5);
        carInfo = [carInfo; str2num(seq_idx) str2num(img_idx) bbox ry];
    end
end

% 
CARS = {};
KPS={};
curr = [num2str(carInfo(1,1)),'_',num2str(carInfo(1,2))];
prev= curr;
x=[];y=[];
cnt =1;
for i=1:size(carInfo,1)
    curr = [num2str(carInfo(i,1)),'_',num2str(carInfo(i,2))];
    if ~strcmp(prev,curr)
        if isempty(x)
           x=vertcat(x,carInfo(i-1,:));
           y=vertcat(y,Keypoints(i-1,:));
        end
        CARS{cnt} = x;
        KPS{cnt} = y;cnt=cnt+1;
        
        x=[];y=[];
        x=vertcat(x,carInfo(i,:));
        y=vertcat(y,Keypoints(i,:));
        prev=curr;
    else
        x=vertcat(x,carInfo(i,:));
        y=vertcat(y,Keypoints(i,:));
    end
end
CARS{cnt}=x;
KPS{cnt}=y;

%}
f=847.630211643;
fx=f;fy=f;cx=960;cy=540;
K = [fx 0 cx; 0 fy cy ; 0 0 1];

% Load the meanshape and the basis vectors.
meanshape = importdata('../../data/meanShape.txt');
vectors = importdata('../../data/vectors.txt');

% Average H, W ,L of the car in metres.
avgCarHeight = 1.5208; avgCarWidth = 1.6362; avgCarLength = 3.8600;

% Keypoint lookup table for binary weight.
kp_lookup= load('../../data/keypoint_visibility_36.mat');kp_lookup = kp_lookup.keypoint_visibility_36;

% To store outputs:
% planeParameters_GT = {};
% planeParameters_Op = {};
% planeParameters_in = {};
% planePoints_in={};
% planePoints_GT={};
% planePoints_Op={};


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


