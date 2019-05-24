% File to write data for shape and pose adjustment.

fID = fopen('../../data/dataVP_KP/data.txt','w');
for i=1:size(cars,1)
    
    
    seq_idx = cars(i,1);img_idx= cars(i,2); id = cars(i,3);
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',img_idx),'.png']);
    tracklets = readLabels(label_dir, seq_idx);
    t_frm = tracklets{img_idx+1};
    CARS = bbox_ry(t_frm,id);
    car= CARS{1};

    fprintf(fID,'%s\n',['/home/sarthaksharma/dataVP_KP/',num2str(i),'.jpg']);
    
    car_bbox = imcrop(im,[car(1),car(2),car(3)-car(1)+1,car(4)-car(2)+1]);
    imwrite(car_bbox,['/home/sarthak/Documents/ICRA_2017/data/dataVP_KP/',num2str(i),'.jpg'])
    
    
    
end
fclose(fID);