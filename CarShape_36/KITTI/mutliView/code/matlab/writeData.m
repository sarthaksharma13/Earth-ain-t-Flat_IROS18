%% Write information about the object. One file for the hourglass and one array with x1 y1 x2 y2 ry for the object from start_frame till end_frame.
fID = fopen('../../data/dataHG/hourglassInput.txt','w');
for img_idx=start_frame:end_frame
    t_frm = tracklets{img_idx+1};
    car = bbox_ry(t_frm,carID);
    car=car{1};
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/ICRA_2018_MOT/KITTITracking/dataset_left/training/image_02/' sprintf('%02d',seq_idx) '/image_0/',sprintf('%06d',img_idx),'.png']);
    fprintf(fID,'%s %f %f %f %f\n',['/home/sarthaksharma/kps/dataHG/',sprintf('%06d',img_idx),'.png'],car(1),car(2),car(3)-car(1)+1,car(4)-car(2)+1);
    imwrite(im,['../../data/dataHG/',sprintf('%06d',img_idx),'.png']);
end
fclose(fID);
