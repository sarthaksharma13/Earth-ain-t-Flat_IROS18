function plotCarPlaneMesh(W_sh,f,c)

Ku = convhulln(W_sh([13:16 13+18:16+18],:));
Kd = convhulln(W_sh([1:13 16:18 1+18:13+18 16+18:18+18],:));
hold on;trisurf(Ku,W_sh([13:16 13+18:16+18],1),W_sh([13:16 13+18:16+18],2),W_sh([13:16 13+18:16+18],3),'edgecolor','none','facecolor',c,'FaceAlpha',0.9);
hold on;trisurf(Kd,W_sh([1:13 16:18 1+18:13+18 16+18:18+18],1),W_sh([1:13 16:18 1+18:13+18 16+18:18+18],2),W_sh([1:13 16:18 1+18:13+18 16+18:18+18],3),'edgecolor','none','facecolor',c,'FaceAlpha',0.9);
% axis equal
% alpha = 0.1;
% P1 = W_sh(6,:);
% P2 = W_sh(7,:);
% P3 = W_sh(24,:);
% v1 = (P1-P2);
% v2 = (P2-P3);
% n = cross(v1, v2);
% n = -n/norm(n);
% d = dot(n,W_sh(6,:));
% d = d+0.25;
% m = mean(W_sh);
visualizeWireframe(W_sh,f,'k','');



end