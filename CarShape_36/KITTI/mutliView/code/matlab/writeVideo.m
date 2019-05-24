images    = cell(3,1);
images{1} = imread('../../1.jpg');
images{2} = imread('../../2.jpg');
images{3} = imread('../../3.jpg');
images{4} = imread('../../4.jpg');
images{5} = imread('../../5.jpg');


% create the video writer with 1 fps
 writerObj = VideoWriter('myVideo.avi');
 writerObj.FrameRate = 1;

 % set the seconds per image
 secsPerImage = [5 5 5 5 5];

 % open the video writer
 open(writerObj);

 % write the frames to the video
 for u=1:length(images)
     % convert the image to a frame
     frame = im2frame(images{u});

     for v=1:secsPerImage(u) 
         writeVideo(writerObj, frame);
     end
 end

 % close the writer object
 close(writerObj);