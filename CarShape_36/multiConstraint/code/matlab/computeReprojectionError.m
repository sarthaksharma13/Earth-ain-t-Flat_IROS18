function error = computeReprojectionError(W3d,K,W2d)

projPnts = K*W3d;
projPnts(1,:)  = projPnts(1,:)./projPnts(3,:);
projPnts(2,:)  = projPnts(2,:)./projPnts(3,:);
imgPnts = projPnts(1:2,:);
error = [];
for i=1:size(imgPnts,2)
   error(i) = norm(imgPnts(:,i)- W2d(:,i));
end


end