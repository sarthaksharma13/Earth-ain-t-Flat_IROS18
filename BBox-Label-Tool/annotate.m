%% Script to annotate instances caputuring different plane profiles.
clc;clear all;close all;
% Number of profiles/labels we want to annotate
numProfiles = 9;

for i=1:numProfiles
    % Read the data for the plane profile.
    D= importdata(['./planeProfiles/planeProfile' num2str(i) '.txt']);
    path = ['./Images/' sprintf('%03s',num2str(i+1))];
    mkdir(path);    
    for j=1:size(D,1)
       seq_idx = D(j,1); img_idx = D(j,2);
       im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(seq_idx) '/RGBLeft/' sprintf('%07s',num2str(img_idx)) '.png']);
       imwrite(im,[path sprintf('/%07s',num2str(seq_idx)) '_' sprintf('%07s',num2str(img_idx)) '.png' ] );
        
    end
    fprintf(['==> Number of images to annotate :',num2str(size(D,1)) '\n' ]);
    fprintf(['Enter class label : ' num2str(i+1) '\n'])
    
%     cmd = 'python main.py'
%     system(cmd);
end