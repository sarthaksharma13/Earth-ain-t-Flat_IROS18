function visualizeSynScene(wireframe,gt_keypoints,titl)


% Left side
edges=[1,2; 2,3; 3,4; 4,5; 5,6; 6,7; 7,8; 8,9; 9,10; 10,11; 11,12; 12,13; 13,14; 14,15; 15,16; 16,1];
% Left side + Right side
edges = [edges;edges + 18];
% Join both the sides.
edges = [edges; 1,19; 2,20; 3,21; 4,22; 7,25; 5,23; 6,24; 7,25; 8,26;];

load('../../data/obsCarRoadPlane.mat');
load('../../data/egoCarRoadPlane.mat');

img=ones(375,1242,3);
% Plot
imshow(img);
hold on;
plot(gt_keypoints(:,1),gt_keypoints(:,2),'k.');
plot(obsCarRPImage1(:,1),obsCarRPImage1(:,2),'rs');
plot(egoCarRPImage1(:,1),egoCarRPImage1(:,2),'bs');
rectangle('Position',[ 558.6644  134.2858   74.0249   63.0642]);
% Plot keypoints
scatter(wireframe(1,:),wireframe(2,:),'g');
colors = distinguishable_colors(size(edges,1), [1, 1, 1]);

% Draw each edge in the plot
for i = 1:size(edges, 1)
    plot(wireframe(1,[edges(i,1), edges(i,2)]), wireframe(2, [edges(i,1), edges(i,2)]), ...
        'LineWidth', 2, 'Color', colors(i,:));
end
title(titl);



end