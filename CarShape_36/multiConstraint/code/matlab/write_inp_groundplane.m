function write_inp_groundplane(K,roadPts3D,R,trans,roadPts2D,N,d)
fID = fopen('../../data/ceres_input_groundPlane.txt','w');

% Number of views
fprintf(fID,'%d\n',1);
% Number of points
fprintf(fID,'%d\n',size(roadPts3D,1));
% K matrix
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',K(1,1),K(1,2),K(1,3),K(2,1),K(2,2),K(2,3),K(3,1),K(3,2),K(3,3));
% 3D road Pts
for i=1:size(roadPts3D,1)
   fprintf(fID,'%f %f %f\n',roadPts3D(i,1),roadPts3D(i,2),roadPts3D(i,3)); 
end

% Rotation for each view
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',R(1,1),R(2,1),R(3,1),R(1,2),R(2,2),R(3,2),R(1,3),R(2,3),R(3,3));

% Translation for each view
fprintf(fID,'%f %f %f\n',trans(1),trans(2),trans(3));

% 2D road Pts
for i=1:size(roadPts2D,1)
   fprintf(fID,'%f %f\n',roadPts2D(i,1),roadPts2D(i,2)); 
end

% Plane parameters
fprintf(fID,'%f %f %f %f\n',N(1),N(2),N(3),d);
fclose(fID);


end