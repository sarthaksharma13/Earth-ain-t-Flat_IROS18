%% Script to select the eigen vectors to be used for the low dimensional subspace representation of the mean wireframe of the car.
% Sarthak Sharma

clc;clear all;close all;
filePath = '../../data/car_annotations_withName.txt';
thr = 0.80;
% numKps = 36;
numKps = 14;

% Load the data.
fprintf('Loading data...\n');
d = importdata(filePath);
d = d.data;
set=[2,3,4,5,7,17,18,20,21,22,23,24,35,36];
D=[];
for i=1:14
    D=[D d(:,(set(i)-1)*3+1:(set(i)-1)*3 +3)];
end
d=D;
% Mean shape
% for i=1:size(d,1);% Take care of the ordering ! Also for keypoint net. Following the order as stated while annotations.
%     currShape = d(i,:);
%     currShape = reshape(currShape,3,numKps)';
%     left = currShape(1:18,:); left = left(18:-1:1,:);
%     right = currShape(19:36,:); right = right(18:-1:1,:);
%     currShape = ([left;right])';
%     d(i,:) = reshape(currShape,1,numKps*3);
% end

meanShape = mean(d);
meanShape = reshape(meanShape,3,numKps)';
% visualizeWireframe3D(meanShape');

%%
% Run the pca
fprintf('Running PCA...\n');
[COEFF, SCORE, LATENT] = pca(d);
% Find the eigen drop
ind = 1;
for i=1:size(LATENT,1)
    if( sum(LATENT(1:i,1))/sum(LATENT) )>= thr
        ind = i;
        break
    end
end

% Take the eigen vectors corresponding to the threshold.
fprintf('Selecting %d eigen vectors\n',ind);
eigVec = COEFF(:,1:ind)';
fileID= fopen('/home/sarthaksharma/Documents/thesis/eigenVals.txt','w');
for i=1:size(LATENT,1)
    fprintf(fileID,'%f ',LATENT(i,1));
end
%%
fprintf('Writing mean shape and basis vectors...\n');
% Save the mean shape to a file
fileID= fopen('/home/sarthak/Documents/CarShape_36/data/meanShape.txt','w');
for i=1:numKps
    fprintf(fileID,'%f %f %f\n',meanShape(i,1),meanShape(i,2),meanShape(i,3));
end

fclose(fileID);

% Save the basis vectors to the file
fileID= fopen('/home/sarthak/Documents/CarShape_36/data/vectors.txt','w');
for i=1:ind
    currEVec = eigVec(i,:);
    for j=1:numKps*3
        fprintf(fileID,'%f',currEVec(1,j));
        if j~=3*numKps
            fprintf(fileID,' ');
        end
    end
    fprintf(fileID,'\n');
    
end
fclose(fileID);





