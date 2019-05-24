function visualizeWireframe(wireframe,figno,c,head)
wireframe=wireframe';
numKps = size(wireframe,2);
figure(figno);
hold on;
% scatter3(wireframe(1,:), wireframe(2,:), wireframe(3,:), repmat(2, 1, numKps));

% scatter3(wireframe(1,1), wireframe(2,1),wireframe(3,1),'filled','k');
% scatter3(wireframe(1,2), wireframe(2,2),wireframe(3,2),'filled','k');
% scatter3(wireframe(1,20), wireframe(2,20),wireframe(3,20),'filled','k');
% scatter3(wireframe(1,19), wireframe(2,19),wireframe(3,19),'filled','k');

% Left side
edges=[3,4; 4,5; 5,6; 6,7; 7,8; 8,9; 9,10; 10,11; 11,12; 12,13; 13,14; 14,15; 15,16; 16,17; 17,18;  18,3];
% Left side + Right side
edges = [edges;edges + 18];
% Join both the sides.
edges = [edges; 18,36; 17,35; 16,34; 15,33; 14,32; 13,31; 12,30; 11,29 ];

for i = 1:size(edges, 1)
    plot3(wireframe(1,[edges(i,1), edges(i,2)]), wireframe(2, [edges(i,1), edges(i,2)]), wireframe(3, [edges(i,1), edges(i,2)]), ...
        'LineWidth', 3.5, 'Color', c);
end

title(head)
     
end
