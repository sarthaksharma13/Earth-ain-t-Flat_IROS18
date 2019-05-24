function [] = visualizeWireframe3D(wireframe,ttl)
% VISUALIZEWIREFRAME3D  Takes in a 3D car wireframe (3 x 36 matrix), and
% plots it in 3D while appropriately connecting vertices
% Number of keypoints for the car class
numKps = size(wireframe,2);

% Generate distinguishable colors with respect to a white background
colors = distinguishable_colors(numKps, [1, 1, 1]);

% Create a scatter plot of the wireframe vertices
scatter3(wireframe(1,:), wireframe(2,:), wireframe(3,:), repmat(2, 1, numKps), colors);
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


% Left side
edges=[3,4; 4,5; 5,6; 6,7; 7,8; 8,9; 9,10; 10,11; 11,12; 12,13; 13,14; 14,15; 15,16; 16,17; 17,18;  18,3];
% Left side + Right side
edges = [edges;edges + 18];
% Join both the sides.
edges = [edges; 18,36; 17,35; 16,34; 15,33; 14,32; 13,31; 12,30; 11,29 ];

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
