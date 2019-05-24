function write_inp_pose(CENTER,avgCarHeight,avgCarWidth,avgCarLength,K,KPS,WF,VEC,RY,kp_lookup,l,N,d)
numCars = size(CENTER,2);
fID = fopen('../../data/ceres_input_singleViewPoseAdjuster.txt','w');
fprintf(fID,'%d\n',numCars);
fprintf(fID,'%d\n',36);
for c=1:numCars
    X3d = CENTER{c};
    fprintf(fID,'%f %f %f\n',X3d(1),X3d(2),X3d(3));
end
fprintf(fID,'%f %f %f\n',avgCarHeight,avgCarWidth,avgCarLength);
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',K(1,1),K(1,2),K(1,3),K(2,1),K(2,2),K(2,3),K(3,1),K(3,2),K(3,3));
for c=1:numCars
    kps=KPS{c};
    for i=1:size(kps,1)
        fprintf(fID,'%f %f\n',kps(i,1),kps(i,2));
    end
end

for c=1:numCars
    theta2 = RY{c};
    % write the weights
    if theta2 <0
        theta2=theta2 + 2*pi;
    end
    theta2=rad2deg(theta2);
    w = kp_lookup(ceil(theta2),:);%/(sum(kp_lookup(ceil(theta2),:)));
    w = w.*0.7 + ~w*0.1 +0.01;
    
    for i=1:length(w)
        fprintf(fID,'%f\n',w(i));
    end
end
for c=1:numCars
    meanshape = WF{c};
    for i=1:size(meanshape,2)
        fprintf(fID,'%f %f %f\n',meanshape(1,i),meanshape(2,i),meanshape(3,i));
    end
end
% Write the number of vectors
fprintf(fID,'%d\n',42);
for c=1:numCars
    vectors = VEC{c};
    % Write the vectors.
    for i=1:size(vectors,1)
        for j=1:size(vectors,2)
            fprintf(fID,'%f',vectors(i,j));
            if j~=size(vectors,2) fprintf(fID,' ');end
        end
        fprintf(fID,'\n');
    end
end

% Write the lambdas
for i=1:size(l,1)
    fprintf(fID,'%f',l(i));
    if i~=size(vectors,1)
        fprintf(fID,' ');
    end
end

fprintf(fID,'\n');
fprintf(fID,'%f %f %f %f\n',N(1),N(2),N(3),d);
fclose(fID);

end
