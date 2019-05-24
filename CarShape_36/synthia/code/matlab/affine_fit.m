function [n, d] = affine_fit(X)

% %the mean of the samples belongs to the plane
% p = mean(X,1);
%
% %The samples are reduced:
% R = bsxfun(@minus,X,p);
% %Computation of the principal directions if the samples cloud
% [V,D] = eig(R'*R);
% %Extract the output from the eigenvectors
% n = V(:,1);
% if n(2) < 0
%    n=-n;
% end
%
% V = V(:,2:end);
% d = abs(dot(n,p));


rng(2);
indx = randi([1 size(X,1)],1,3);
% Get any three points from plane
pt1 = X(indx(1),:);
pt2 = X(indx(2),:);
pt3 = X(indx(3),:);

% Find cross product using point 1 as pivot
n = cross(pt3-pt1,pt2-pt1);

if n(2) < 0
    n=-n;
end
n=n/norm(n);
d= dot(n,pt1);


end