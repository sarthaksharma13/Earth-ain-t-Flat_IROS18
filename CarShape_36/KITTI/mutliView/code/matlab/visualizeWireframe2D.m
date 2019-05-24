function [] = visualizeWireframe2D(img, wireframe)
% VISUALIZEWIREFRAME2D  Takes in a 2D car wireframe (2 x 14 matrix), and
% plots it in 2D (on a given image) while appropriately connecting vertices


% Declare global variables
%globals;

% Number of keypoints for the car class
numKps = size(wireframe,2);

% Generate distinguishable colors with respect to a white background
colors = distinguishable_colors(numKps, [0, 0, 0]);

% Display the image
imshow(img);
% Hold on, to plot the wireframe
hold on;

% Create a scatter plot of the wireframe vertices
scatter(wireframe(1,:), wireframe(2,:), repmat(20, 1, numKps), colors, 'filled');
% scatter3(wireframe(1,:), wireframe(2,:), wireframe(3,:), 'filled', 'MarkerFaceColor', colors);

% Get car part names
%load(fullfile(localDataDir, 'partNames', 'car'));

% Plot text labels for car keypoints, using distinguishable colors
% text(wireframe(1,1), wireframe(2,1), 'L\_F\_WheelCenter', 'color', colors(1,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,2), wireframe(2,2), 'R\_F\_WheelCenter', 'color', colors(2,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,3), wireframe(2,3), 'L\_B\_WheelCenter', 'color', colors(3,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,4), wireframe(2,4), 'R\_B\_WheelCenter', 'color', colors(4,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,5), wireframe(2,5), 'L\_HeadLight', 'color', colors(5,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,6), wireframe(2,6), 'R\_HeadLight', 'color', colors(6,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,7), wireframe(2,7), 'L\_TailLight', 'color', colors(7,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,8), wireframe(2,8), 'R\_TailLight', 'color', colors(8,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,9), wireframe(2,9), 'L\_SideViewMirror', 'color', colors(9,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,10), wireframe(2,10), 'R\_SideViewMirror', 'color', colors(10,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,11), wireframe(2,11), 'L\_F\_RoofTop', 'color', colors(11,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,12), wireframe(2,12), 'R\_F\_RoofTop', 'color', colors(12,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,13), wireframe(2,13), 'L\_B\_RoofTop', 'color', colors(13,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);
% text(wireframe(1,14), wireframe(2,14), 'R\_B\_RoofTop', 'color', colors(14,:), 'FontSize', 10, 'BackgroundColor', [0, 0, 0]);

% Car parts (keypoints) are indexed in the following manner
% 1  ->  'L_F_WheelCenter'
% 2  ->  'R_F_WheelCenter'
% 3  ->  'L_B_WheelCenter'
% 4  ->  'R_B_WheelCenter'
% 5  ->  'L_HeadLight'
% 6  ->  'R_HeadLight'
% 7  ->  'L_TailLight'
% 8  ->  'R_TailLight'
% 9  ->  'L_SideViewMirror'
% 10 ->  'R_SideViewMirror'
% 11 ->  'L_F_RoofTop'
% 12 ->  'R_F_RoofTop'
% 13 ->  'L_B_RoofTop'
% 14 ->  'R_B_RoofTop'

% L_F_RoofTop -> R_F_RoofTop -> R_B_RoofTop -> L_B_RoofTop
edges = [11, 12; 12, 14; 14, 13; 13, 11;];
% L_HeadLight -> R_HeadLight -> R_TailLight -> L_TailLight
edges = [edges; 5, 6; 6, 8; 8, 7; 7, 5];
% L_Headlight -> L_F_RoofTop
edges = [edges; 5, 11];
% R_HeadLight -> R_F_RoofTop
edges = [edges; 6, 12];
% L_TailLight -> L_B_RoofTop
edges = [edges; 7, 13];
% R_TailLight -> R_B_RoofTop
edges = [edges; 8, 14];
% L_F_WheelCenter -> R_F_WheelCenter -> R_B_WheelCenter -> L_B_WheelCenter
edges = [edges; 1, 2; 2, 4; 4, 3; 3, 1];
% L_HeadLight -> L_F_WheelCenter
edges = [edges; 5, 1];
% R_HeadLight -> R_F_WheelCenter
edges = [edges; 6, 2];
% L_TailLight -> L_B_WheelCenter
edges = [edges; 7, 3];
% R_TailLight -> R_B_WheelCenter
edges = [edges; 8, 4];
% L_SideViewMirror -> L_HeadLight
edges = [edges; 9, 5];
% R_SideViewMirror -> R_HeadLight
edges = [edges; 10, 6];
% % L_SideViewMirror -> R_SideViewMirror
% edges = [edges; 9, 10];
% % L_SideViewMirror -> L_F_RoofTop
% edges = [edges; 9, 11];
% % R_SideViewMirror -> R_F_RoofTop
% edges = [edges; 10, 12];

% Generate distinguishable colors (equal to the number of edges). The
% second parameter to the function is the background color.
colors = distinguishable_colors(size(edges,1), [1, 1, 1]);

% Draw each edge in the plot
for i = 1:size(edges, 1)
    plot(wireframe(1,[edges(i,1), edges(i,2)]), wireframe(2, [edges(i,1), edges(i,2)]), ...
        'LineWidth', 2, 'Color', colors(i,:));
end


% % L_F_RoofTop -> R_F_RoofTop -> R_B_RoofTop -> L_B_RoofTop
% plot3(wireframe(1,[11,12]), wireframe(2,[11,12]), wireframe(3,[11,12]), 'LineWidth', 2, 'Color', colors(1,:));
% plot3(wireframe(1,[12,14]), wireframe(2,[12,14]), wireframe(3,[12,14]));
% plot3(wireframe(1,[14,13]), wireframe(2,[14,13]), wireframe(3,[14,13]));
% plot3(wireframe(1,[13,11]), wireframe(2,[13,11]), wireframe(3,[13,11]));
% % L_HeadLight -> R_HeadLight -> R_TailLight -> L_TailLight
% plot3(wireframe(1,[5,6]), wireframe(2,[5,6]), wireframe(3,[5,6]));
% plot3(wireframe(1,[6,8]), wireframe(2,[6,8]), wireframe(3,[6,8]));
% plot3(wireframe(1,[8,7]), wireframe(2,[8,7]), wireframe(3,[8,7]));
% plot3(wireframe(1,[7,5]), wireframe(2,[7,5]), wireframe(3,[7,5]));
% % L_Headlight -> L_F_RoofTop
% plot3(wireframe(1,[5,11]), wireframe(2,[5,11]), wireframe(3,[5,11]));
% % R_HeadLight -> R_F_RoofTop
% plot3(wireframe(1,[6,12]), wireframe(2,[6,12]), wireframe(3,[6,12]));
% % L_TailLight -> L_B_RoofTop
% plot3(wireframe(1,[7,13]), wireframe(2,[7,13]), wireframe(3,[7,13]));
% % R_TailLight -> R_B_RoofTop
% plot3(wireframe(1,[8,14]), wireframe(2,[8,14]), wireframe(3,[8,14]));
% % L_F_WheelCenter -> R_F_WheelCenter -> R_B_WheelCenter -> L_B_WheelCenter
% plot3(wireframe(1,[1,2]), wireframe(2,[1,2]), wireframe(3,[1,2]));
% plot3(wireframe(1,[2,4]), wireframe(2,[2,4]), wireframe(3,[2,4]));
% plot3(wireframe(1,[4,3]), wireframe(2,[4,3]), wireframe(3,[4,3]));
% plot3(wireframe(1,[3,1]), wireframe(2,[3,1]), wireframe(3,[3,1]));
% % L_HeadLight -> L_F_WheelCenter
% plot3(wireframe(1,[5,1]), wireframe(2,[5,1]), wireframe(3,[5,1]));
% % R_HeadLight -> R_F_WheelCenter
% plot3(wireframe(1,[6,2]), wireframe(2,[6,2]), wireframe(3,[6,2]));
% % L_TailLight -> L_B_WheelCenter
% plot3(wireframe(1,[7,3]), wireframe(2,[7,3]), wireframe(3,[7,3]));
% % R_TailLight -> R_B_WheelCenter
% plot3(wireframe(1,[8,4]), wireframe(2,[8,4]), wireframe(3,[8,4]));
% % L_SideViewMirror -> L_HeadLight
% plot3(wireframe(1,[9,5]), wireframe(2,[9,5]), wireframe(3,[9,5]));
% % R_SideViewMirror -> R_HeadLight
% plot3(wireframe(1,[10,6]), wireframe(2,[10,6]), wireframe(3,[10,6]));

% Plot title
title('2D Projection of the Car Wireframe');

hold off;

end
