% File to write data for shape and pose adjustment.

fID = fopen('../../data/dataVP/data.txt','w');
for i=1:size(carInfo,1)
    
    seq_idx = carInfo(i,1);img_idx= carInfo(i,2); bbox = carInfo(i,3:6);
    
    % Load the image corresponding to it.
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(seq_idx) '/RGBLeft/' sprintf('%07s',num2str(img_idx)) '.png']);
    
    
    fprintf(fID,'%s\n',['/home/sarthaksharma/predictKps/dataHG/',num2str(i),'.jpg']);
    
    car_bbox = imcrop(im,[bbox(1),bbox(2),bbox(3)-bbox(1)+1,bbox(4)-bbox(2)+1]);
    imwrite(car_bbox,['../../data/dataVP/',num2str(i),'.jpg'])
    
    
    
end
fclose(fID);