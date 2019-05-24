%% Write information about the object. One file for the hourglass and one array with x1 y1 x2 y2 ry for the object from start_frame till end_frame.
carInfo =[];
fID = fopen('../../data/dataHG/hourglassInput.txt','w');

cam =2 ;
root_dir = '/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT';
label_dir = fullfile([root_dir '/KITTITracking/'], sprintf('label_%02d/',cam));
image_dir = fullfile([root_dir '/KITTITracking/dataset_left/training/' sprintf('image_%02d/%02d/image_0/',cam, seq_idx)] );
tracklets = readLabels(label_dir, seq_idx);
for img_idx=start_frame : end_frame
    t_frm = tracklets{img_idx+1};
    car = bbox_ry(t_frm,carID);
    car=car{1};
    carInfo = [carInfo;car(1) car(2) car(3) car(4) car(5)];
    fprintf(fID,'%s %f %f %f %f\n',['/home/sarthaksharma/kps/dataHG/',sprintf('%06d',img_idx),'.png'],car(1),car(2),car(3)-car(1)+1,car(4)-car(2)+1);
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',img_idx),'.png']);
    %imshow(im);hold on;rectangle('Position',[car(1),car(2),car(3)-car(1)+1,car(4)-car(2)+1]);
    imwrite(im,['../../data/dataHG/',sprintf('%06d',img_idx),'.png']);
    
end

fclose(fID);
