clc;clear all;close all;
a=load('../../data/synthia_2_54_90.mat');
a=a.bbmat;
v = values(a);
k = keys(a);
cnt=1;
fID = fopen('../../data/dataVP/data.txt','w');

for i=1:numel(keys(a))
    cars = v(i);
    imgName = k(i);
    splt = strsplit(imgName{1},'_');
    
%     ss=strcat('../../data/img/', imgName, '.png');
%     im = imread(ss{1});
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(splt{1}) '/RGBLeft/' sprintf('%07s',num2str(splt{2})) '.png']);
    cars=cars{1};
    for j=1:size(cars,1)
        bbox = cars(j,1:4); ry = cars(j,5);
        car_bbox = imcrop(im,[bbox(1),bbox(2),bbox(3)-bbox(1)+1,bbox(4)-bbox(2)+1]);
        imwrite(car_bbox,['../../data/dataVP/',num2str(cnt),'.jpg']);
        fprintf(fID,'%s\n',['/home/sarthaksharma/predictKps/dataHG/',num2str(cnt),'.jpg']);
        cnt=cnt+1;
    end
end

fclose(fID);