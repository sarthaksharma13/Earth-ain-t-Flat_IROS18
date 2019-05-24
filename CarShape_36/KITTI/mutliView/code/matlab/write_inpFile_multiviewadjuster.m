fID = fopen('../../data/ceres_input_multiViewAdjuster.txt','w');
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
    for j=1:size(SHAPEbeforesingleView{i},1)
        fprintf(fID,'%f %f %f\n',SHAPEbeforesingleView{i}(j,1),SHAPEbeforesingleView{i}(j,2),SHAPEbeforesingleView{i}(j,3));
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
% write the lambdas
ll=LaftersingleView{1};
for i=2:numViews
    ll = ll + LaftersingleView{2};
end
ll=ll/numViews;
fprintf(fID,'%f %f %f %f %f\n',ll(1),ll(2) ,ll(3),ll(4),ll(5));
%write pose and shape for each view.
for i=1:numViews
    pose = POSEafterPnP{i};
    fprintf(fID,'%f %f %f %f %f %f %f %f %f\n',pose(1),pose(2) ,pose(3), pose(4),pose(5),pose(6),pose(7),pose(8),pose(9));
end
for i=1:numViews
    pose = POSEafterPnP{i};
    fprintf(fID,'%f %f %f\n',pose(10),pose(11) ,pose(12));
end

