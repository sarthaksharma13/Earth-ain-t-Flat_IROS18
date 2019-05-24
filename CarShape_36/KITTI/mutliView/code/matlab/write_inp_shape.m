function write_inp_shape(X3d,avgCarHeight,avgCarWidth,avgCarLength,K,kps,meanshape,vectors,theta2,kp_lookup)
pose = importdata('./ceres_output_singleViewPoseAdjuster.txt');

fID = fopen('../../data/ceres_input_singleViewShapeAdjuster.txt','w');
fprintf(fID,'%d %d %d\n',1,14,14);
fprintf(fID,'%f %f %f\n',X3d(1),X3d(2),X3d(3));
fprintf(fID,'%f %f %f\n',avgCarHeight,avgCarWidth,avgCarLength);
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',K(1,1),K(1,2),K(1,3),K(2,1),K(2,2),K(2,3),K(3,1),K(3,2),K(3,3));
for i=1:size(kps,1)
    fprintf(fID,'%f %f\n',kps(i,1),kps(i,2));
end

wV = kp_lookup(floor(theta2),:)/(sum(kp_lookup(floor(theta2),:)));
wHG = kps(:,3);
w = 0.7*(wV') + 0.3*wHG ;
for i=1:length(w)
    fprintf(fID,'%f\n',w(i));
end

%fprintf(fID,'%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n',0.125,0.000001,0.125,0.000001,0.000001,0.000001,0.125,0.125,0.125,0.000001,0.125,0.000001,0.125,0.125);

for i=1:size(meanshape,2)
    fprintf(fID,'%f %f %f\n',meanshape(1,i),meanshape(2,i),meanshape(3,i));
end
for i=1:size(vectors,1)
    for j=1:size(vectors,2)
        fprintf(fID,'%f',vectors(i,j));
        if j~=size(vectors,2) fprintf(fID,' ');end
    end
    fprintf(fID,'\n');
end
fprintf(fID,'%f %f %f %f %f',0.25, 0.27 ,0.01, -0.08, -0.05);
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',pose(1),pose(2) ,pose(3), pose(4),pose(5),pose(6),pose(7),pose(8),pose(9));
fprintf(fID,'%f %f %f\n',pose(10),pose(11) ,pose(12));

fclose(fID);

end
