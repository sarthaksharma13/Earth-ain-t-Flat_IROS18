images={};
folder = '../../resultsCarShape/10/';
nimages = dir([folder '*.jpg']);
for i=1:size(nimages,1)
    images{i} = imread([folder num2str(i) '.jpg']);
    
end


writerObj = VideoWriter([ '../../temp.avi']);
writerObj.FrameRate = 5;

% set the seconds per image

% open the video writer
open(writerObj);

% write the frames to the video
for i=1:size(nimages,1)
    p=images{i};
    
% %     im=imread([D p.name]);
    % convert the image to a frame
    frame = im2frame(p);
    frame = imresize(frame,[1600 541]);
    
    writeVideo(writerObj, frame);
    
end

% close the writer object
close(writerObj);