label = 1;
d = dir([ '../../data/Labels/' sprintf('%03d',label) '/*.txt' ]);
for i=1:1
    cf=d(i).name
    bbox = importdata(['../../data/Labels/' sprintf('%03d',label) '/' cf]) 
    a = strsplit(cf,'_');b=strsplit(a{2},'.');
    seq_idx = str2num(a{1})
    img_idx = str2num(b{1})
    im = imread(['/media/sarthak/Seagate Backup Plus Drive/SYNTHIA-SF/SEQ' num2str(seq) '/RGBLeft/' sprintf('%07s',num2str(img_idx)) '.png']);
    imshow(im);hold on;
    for j=1:size(bbox,1)
       rectangle('Position',[bbox(j,1),bbox(j,2),bbox(j,3)-bbox(j,1)+1,bbox(j,4)-bbox(j,2)+1],'LineWidth',2,'EdgeColor','r');
       text(bbox(j,1), bbox(j,2),num2str(j));
        
    end
    pause(2)
end
