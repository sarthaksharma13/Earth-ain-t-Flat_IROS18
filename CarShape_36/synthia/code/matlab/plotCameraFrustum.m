% function to plot camera frustrums and trajectory plots only the left 
% camera as 3D is computed in left camera's frame
%
% INPUT:
%
% T 	  : 4x4 camera pose
% camCol   : color of the camera to be plotted
% camScale : how big you want the camera to look in the plot
%

function plotCameraFrustum(T,camCol, camScale,fno)

	% points for ploting camera frustrum; points are in homogeneous
	% coordinates
	camFrustrum = [-1 -1  1  1;
			 	 1 -1  1  1;
			 	 1  1  1  1;
				-1  1  1  1;
				-1 -1  1  1;
			 	 0  0  0  1;
			 	 1 -1  1  1;
			 	 1  1  1  1;
			 	 0  0  0  1;
			 	-1  1  1  1] ;		
	
	camFrustrum(:,1:3) =camFrustrum(:,1:3) * camScale;  
	
	figure(1);
	
	camColor = camCol;
		
	% apply transform to the frustrum and plot it
	transformedCamFrustrum = (T*camFrustrum')';
    
    figure(fno);
    hold on;
	plot3(transformedCamFrustrum(:,1), transformedCamFrustrum(:,2), transformedCamFrustrum(:,3), strcat('-',camColor), 'LineWidth', 2);	
	
end
