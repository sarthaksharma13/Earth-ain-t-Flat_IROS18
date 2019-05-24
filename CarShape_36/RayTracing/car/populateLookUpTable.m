%% Script to offline populate a lookup table for various azimuths for 36 keypoint model. 
% Sarthak Sharma
clc;clear all;close all;
keypoint_visibility_36 =[];

for i=0:359
    cmd = ['./drawCAD ',num2str(i)];
    system(cmd);
    keypoint_visibility_36 = [keypoint_visibility_36;importdata('./visibility_output.txt')];
end