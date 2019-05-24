clear all;close all;
%% Run the adjustment code
clc;
d = dir('../../data/modelFiles/*.txt');
nFiles = size(d,1);
for i=1:nFiles

    srcFile = ['../../data/modelFiles/' d(i).name];
    dstFile = '../../data/left.txt';
    
    % Copy file for that model as the input required by ceres.
    system(['cp ' srcFile ' ' dstFile]);
    
    % Run shape and pose adjustment
    system('./../../build/adjuster');
        
    srcFile = '../../adjusted_l.txt';
    dstFile = ['../../data/adjustedModelFiles/' d(i).name];
    % Copy the adjusted file to the output folder
    system(['cp ' srcFile ' ' dstFile]);
    fprintf(['Done for ' num2str(i) '/' num2str(nFiles) '\n']);
    
end


%% Visualize the left half.
clc;
data = textscan(fopen('../../data/left.txt','r'),'%f');
adjusted = importdata('../../adjusted_l.txt');

% Load the required data for calculating the reprojection error.
nViews = data{1}(1,1); nPoints = data{1}(2,1);
K = data{1}(3:11,1); K = reshape(K,3,3)';
wireframe =  data{1}(12 : 12 + nPoints*3 -1,1);
wireframe = reshape(wireframe,3,nPoints)';
R = data{1}(1 + 1 + 9 + nPoints*3 + 1:1 + 1 + 9 + nPoints*3 + nViews*9,1) ;
T = data{1}(1 + 1 + 9 + nPoints*3 + nViews*9 + 1:1 + 1 + 9 + nPoints*3 + nViews*9 +  nViews*3,1) ;
R = reshape(R,9,nViews)'; T = reshape(T,3,nViews)';
imagePnts = data{1}(end - 2*nViews*nPoints +1:end,:);imagePnts = reshape(imagePnts,2,nViews*nPoints)';
for view =1:nViews
    
    % Get R and T for that view.
    Rv = R(view,:);Rv=reshape(Rv,3,3)'; % Matrices given in row major order.
    Tv = T(view,:);
       
    % For each view,project the triangulated 3D points( before adjustment).
    projectTriangV = K*(Rv'*(wireframe' - repmat(Tv',1,nPoints))); 
    projectTriangV(1,:) = projectTriangV(1,:)./projectTriangV(3,:);
    projectTriangV(2,:) = projectTriangV(2,:)./projectTriangV(3,:);
    projectTriangV= projectTriangV(1:2,:)';
    
    % For each view project the 2D points from the adjusted 3D points.
    adjustedV =  K*(Rv'*(adjusted' - repmat(Tv',1,nPoints)));
    adjustedV(1,:) = adjustedV(1,:)./adjustedV(3,:);
    adjustedV(2,:) = adjustedV(2,:)./adjustedV(3,:);
    adjustedV = adjustedV(1:2,:)';
    
    % The given image points.
    imageV = imagePnts((view-1)*nPoints +1 :(view-1)*nPoints +nPoints,:);

    plot(imageV(:,1),-imageV(:,2),'-s');axis equal;hold on;
    plot(adjustedV(:,1),-adjustedV(:,2),'-sr');axis equal;hold on;
    plot(projectTriangV(:,1),-projectTriangV(:,2),'-s');axis equal;hold on;
    legend('Initial image points', 'after ceres adjustment','before adjustment');
    pause(15);close all;

end


