


fID = fopen('../../data/ceresInputFile.txt','w');
fprintf(fID,'%d %d %d\n',numViews,14,14);
fprintf(fID,'%f %f %f\n',avgCarHeight,avgCarWidth,avgCarLength);
fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',K(1,1),K(1,2),K(1,3),K(2,1),K(2,2),K(2,3),K(3,1),K(3,2),K(3,3));
% for each view write : car center,kps, weights,shape,vectors
for i=1:numViews
    fprintf(fID,'%f %f %f\n',CENTER{i}(1),CENTER{i}(2),CENTER{i}(3));
end
for i=1:numViews
    for j=1:size(KPS{i},1)
        fprintf(fID,'%f %f\n',KPS{i}(j,1),KPS{i}(j,2));
    end
end
for i=1:numViews
    for j=1:size(KPS{i},1)
        fprintf(fID,'%f\n',KPS{i}(j,3));
    end
end
for i=1:numViews
    for j=1:size(SHAPE{i},2)
        fprintf(fID,'%f %f %f\n',SHAPE{i}(1,j),SHAPE{i}(2,j),SHAPE{i}(3,j));
    end
end
for i=1:numViews
    for j=1:size(VECTORS{i},1)
        for k=1:size(VECTORS{i},2)
            fprintf(fID,'%f',VECTORS{i}(j,k));
            if k~=size(VECTORS{i},2) fprintf(fID,' ');end
        end
        fprintf(fID,'\n');
    end
end



% write the lambdas.
fprintf(fID,'%f %f %f %f %f\n',0.25,0.27,0.01,-0.08,-0.05);
fclose(fID);



