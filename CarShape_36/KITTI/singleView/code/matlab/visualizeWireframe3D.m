function [] = visualizeWireframe3D(wireframe,ttl)
% VISUALIZEWIREFRAME3D  Takes in a 3D car wireframe (3 x 36 matrix), and
% plots it in 3D while appropriately connecting vertices
% Number of keypoints for the car class
numKps = size(wireframe,2);

% Generate distinguishable colors with respect to a white background
colors = distinguishable_colors(numKps, [1, 1, 1]);

% Create a scatter plot of the wireframe vertices
scatter3(wireframe(1,:), wireframe(2,:), wireframe(3,:), repmat(2, 1, numKps), colors);
%axis([-10 10 -5 5 -5 5]);
% view(60,180);
% Hold on, to plot the edges
hold on;
axis equal;hold on;
% Axis labels
xlabel('X');
ylabel('Y');
zlabel('Z');

% Generate distinguishable colors with respect to a black background
buf = 7; % Number of extra colors to generate
colors = distinguishable_colors(numKps+buf, [1, 1, 1]);
% Randomly permute colors
rng(5);
perm = randperm(numKps+buf);
colors(1:end, :) = colors(perm, :);

% Plot text labels for car keypoints, using distinguishable colors
bgColor = [0.9, 0.9, 0.9];
text(wireframe(1,17), wireframe(2,17), wireframe(3,17), 'L\_F\_WheelCenter', 'color', colors(17,:), 'BackgroundColor', bgColor);
text(wireframe(1,35), wireframe(2,35), wireframe(3,35), 'R\_F\_WheelCenter', 'color', colors(35,:), 'BackgroundColor', bgColor);
 text(wireframe(1,18), wireframe(2,18), wireframe(3,18), 'L\_B\_WheelCenter', 'color', colors(18,:), 'BackgroundColor', bgColor);
 text(wireframe(1,36), wireframe(2,36), wireframe(3,36), 'R\_B\_WheelCenter', 'color', colors(36,:), 'BackgroundColor', bgColor);
% text(wireframe(1,1), wireframe(2,1), wireframe(3,1), 'L\_HeadLight', 'color', colors(1,:), 'BackgroundColor', bgColor);
% text(wireframe(1,19), wireframe(2,19), wireframe(3,19), 'R\_HeadLight', 'color', colors(19,:), 'BackgroundColor', bgColor);
% text(wireframe(1,8), wireframe(2,8), wireframe(3,8), 'L\_TailLight', 'color', colors(8,:), 'BackgroundColor', bgColor);
% text(wireframe(1,26), wireframe(2,26), wireframe(3,26), 'R\_TailLight', 'color', colors(26,:), 'BackgroundColor', bgColor);
% text(wireframe(1,4), wireframe(2,4), wireframe(3,4), 'L\_F\_RoofTop', 'color', colors(4,:), 'BackgroundColor', bgColor);
% text(wireframe(1,22), wireframe(2,22), wireframe(3,22), 'R\_F\_RoofTop', 'color', colors(22,:), 'BackgroundColor', bgColor);
% text(wireframe(1,5), wireframe(2,5), wireframe(3,5), 'L\_B\_RoofTop', 'color', colors(5,:), 'BackgroundColor', bgColor);
% text(wireframe(1,23), wireframe(2,23), wireframe(3,23), 'R\_B\_RoofTop', 'color', colors(23,:), 'BackgroundColor', bgColor);

% Left side
edges=[1,2; 2,3; 3,4; 4,5; 5,6; 6,7; 7,8; 8,9; 9,10; 10,11; 11,12; 12,13; 13,14; 14,15; 15,16; 16,1];
% Left side + Right side
edges = [edges;edges + 18];
% Join both the sides.
edges = [edges; 1,19; 2,20; 3,21; 4,22; 7,25; 5,23; 6,24; 7,25; 8,26;];

% Generate distinguishable colors (equal to the number of edges). The
% second parameter to the function is the background color.
colors = distinguishable_colors(size(edges,1), [1, 1, 1]);

% Draw each edge in the plot
for i = 1:size(edges, 1)
    plot3(wireframe(1,[edges(i,1), edges(i,2)]), wireframe(2, [edges(i,1), edges(i,2)]), wireframe(3, [edges(i,1), edges(i,2)]), ...
        'LineWidth', 2, 'Color', colors(i,:));
end

% Plot title
title(['3D Wireframe of the car',ttl]);

hold off;

end
