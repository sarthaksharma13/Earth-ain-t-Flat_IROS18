%% Script that computes the plane parameters according to our formulation. Written for SYNTHIA dataset. Take in a bbox coordinates, and samples the points inside
% the bounding box which lie on the plane(using the segmentation provided
% by SYNTHIA-SF. For those road points, we get the corresponding 3D points
% using the depth provided byt SYNTHIA-SF. Those 3D points are used to form
% a plane and the parameters of the plane are returned. If multiple bboxes
% are given in info, then they are all assumed to be in the same sequence.

function [Pts3D,Pts2D] = get3DPoints(typ,info)
% Some globals
f=847.630211643;
fx=f;fy=f;cx=0;cy=0;
w = 1920;
h = 1080;
seq = info(1,1); img_idx= info(1,2);
%% Segmentation
% Read the segmentation image and segment out the points that lie inside
% the give bbox and are road points
ID_im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(seq) '/GTLeft/' sprintf('%07s',num2str(img_idx)) '.png']);
dims = size(ID_im);
ID_im = int32(ID_im);
labelMatrix = ID_im(:,:,1);
[x, y] = meshgrid(1:dims(2), 1:dims(1));
x = reshape(x, dims(1)*dims(2), 1);
y = reshape(y, dims(1)*dims(2), 1);
roadPixels= (labelMatrix) == 1; % gives a logical output 0 or 1. Use that to filter out the non road points
roadPixels = reshape(roadPixels,dims(1)*dims(2),1);
xr = (roadPixels).*x;
yr = (roadPixels).*y;
xr=xr(find(xr(:,1)~=0),1); % columns
yr=yr(find(yr(:,1)~=0),1); % rows

if typ == 1
    % We would return only the 3D Points of the road of 'interest'
    % Choose only those points which lie in the bbox we want
    bbox=info(1,3:6);
    indx = find(xr > bbox(1) & xr < bbox(3));
    indy = find(yr > ((bbox(2)+bbox(4))/2) & yr < bbox(4));
    xr = xr(intersect(indx,indy),1);
    yr = yr(intersect(indx,indy),1);
    x_final = xr;
    y_final = yr;
elseif typ==2
    % Here we would be return the entire road in the scene, plus all the 3D
    % points of the car(s)
    pixels = (labelMatrix) == 1 | (labelMatrix) ==14;
%     pixels = (labelMatrix) == 1 ;
    pixels = reshape(pixels,dims(1)*dims(2),1);
    x_final =pixels.*x;
    y_final =pixels.*y;
    x_final=x_final(find(x_final(:,1)~=0),1); % columns
    y_final=y_final(find(y_final(:,1)~=0),1); % rows
    
else
    % Here we would be returning only the 3D points of the car we want in
    % the given bbox
    bbox=info(1,3:6);
    
    pixels = (labelMatrix ==14);
    pixels = reshape(pixels,dims(1)*dims(2),1);
    x_final =pixels.*x; y_final =pixels.*y;
    
    x_final=x_final(find(x_final(:,1)~=0),1); % columns
    y_final=y_final(find(y_final(:,1)~=0),1); % rows
    indx = find(x_final > bbox(1) & x_final < bbox(3));
    indy = find(y_final > bbox(2) & y_final < bbox(4));
    x_final = x_final(intersect(indx,indy),1);
    y_final = y_final(intersect(indx,indy),1);
end

%% Getting depth
% read the image
D_im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(seq) '/DepthLeft/' sprintf('%07s',num2str(img_idx)) '.png']);
D_im = int32(D_im);
depthMatrix = double( D_im(:,:,1) + (D_im(:,:,2) * 256) + (D_im(:,:,3) * 256 * 256) ) / double((256 * 256 * 256) - 1) * 1000;

%% For each of the ground point, compute its 3D location

idx = sub2ind(size(depthMatrix),y_final,x_final); % To index the matrix linearly, first rows(yr) then columns(xr).
depth = depthMatrix(idx);
Z = depth;
Y = ((y_final-h/2)/f).*depth;
X = ((x_final - w/2)/f).*depth;
Pts3D = [X Y Z];
Pts2D = [x_final y_final];
% pcshow(Pts3D)


end