function write_inp_shape(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps,meanshape,vectors,theta2,kp_lookup,l)

pose = importdata('../../ceres_output_singleViewPoseAdjuster.txt');

fID = fopen('../../data/ceres_input_singleViewShapeAdjuster.txt','w');
fprintf(fID,'%d\n',36);
fprintf(fID,'%f %f %f\n',X3d(1),X3d(2),X3d(3));
fprintf(fID,'%f %f %f\n',avgCarHeight,avgCarWidth,avgCarLength);
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',K(1,1),K(1,2),K(1,3),K(2,1),K(2,2),K(2,3),K(3,1),K(3,2),K(3,3));
for i=1:size(kps,1)
    fprintf(fID,'%f %f\n',kps(i,1),kps(i,2));
end

% write the weights
if theta2 <0
    theta2=theta2 + 2*pi;
end
theta2=rad2deg(theta2);
wV = kp_lookup(ceil(theta2),:)/(sum(kp_lookup(ceil(theta2),:)));
wV = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1];
% wHG = kps(:,3);
w = 0.7*(wV') + 0.01 ;
for i=1:length(w)
    fprintf(fID,'%f\n',1);
end

% Write the meanshape
for i=1:size(meanshape,2)
    fprintf(fID,'%f %f %f\n',meanshape(1,i),meanshape(2,i),meanshape(3,i));
end
% Write the number of vectors
fprintf(fID,'%d\n',size(vectors,1));

% Write the vectors
for i=1:size(vectors,1)
    for j=1:size(vectors,2)
        fprintf(fID,'%f',vectors(i,j));
        if j~=size(vectors,2) fprintf(fID,' ');end
    end
    fprintf(fID,'\n');
end
% Write the lambdas

for i=1:size(vectors,1)
    fprintf(fID,'%f',l(i));
    if i~=size(vectors,1)
        fprintf(fID,' ');
    end
end
fprintf(fID,'\n');
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',pose(1),pose(2) ,pose(3), pose(4),pose(5),pose(6),pose(7),pose(8),pose(9));
fprintf(fID,'%f %f %f\n',pose(10),pose(11) ,pose(12));

fclose(fID);

end
