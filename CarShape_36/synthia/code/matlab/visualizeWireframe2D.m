function [] = visualizeWireframe2D(wireframe,figN,c)
% VISUALIZEWIREFRAME2D  Takes in a 2D car wireframe (2 x 14 matrix), and
% plots it in 2D (on a given image) while appropriately connecting vertices

% Number of keypoints for the car class
numKps = size(wireframe,2);

% Generate distinguishable colors with respect to a white background
colors = distinguishable_colors(numKps, [0, 0, 0]);

% Display the image
figure(figN);
hold on;

% Create a scatter plot of the wireframe vertices
% scatter(wireframe(1,:), wireframe(2,:), repmat(20, 1, numKps), c, 'filled');


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
    plot(wireframe(1,[edges(i,1), edges(i,2)]), wireframe(2, [edges(i,1), edges(i,2)]), ...
        'LineWidth', 1.25, 'Color',c); % colors(i,:)
end

% Plot title
title('2D Projection of the Car Wireframe');

hold off;

end
