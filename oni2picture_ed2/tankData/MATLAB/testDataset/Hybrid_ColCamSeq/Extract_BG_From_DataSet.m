function [color_BG, depth_BG] = Extract_BG_From_DataSet(Folder_Name, D_FolderName, RGB_FolderName, depth_ext, color_ext)
% [color_BG, depth_BG] = Extract_BG_From_DataSet(Folder_Name, RGB_FolderName, D_FolderName)
% Run this file in the root dir of the RGB-D dataset to extract:
%   1. color_BG: intensity (Y component) of the background
%   2. depth_BG: depth map of the background
% e.g.: [color_BG, depth_BG] = Extract_BG_From_DataSet('ColCamSeq');
% - Folder_Name can be 'ColCamSeq'/ 'DCamSeq' / 'GenSeq' / 'movedBG' / 'ShSeq'
%   when using default folder names of color and depth sequences
% - For 'stereoSeq':
%   D_FolderName = 'disparityData';
% * Created by Chongyu Chen @2017-08-28
% * Note: for 'ColCamSeq' dataset, "depth_934.png" does not exist;

if nargin<1, Folder_Name = 'ShSeq'; end
if nargin<2, D_FolderName = 'depthData'; end
if nargin<3, RGB_FolderName = 'colorData'; end
if nargin<4, depth_ext = 'png'; end    % for 'stereoSeq', depth_ext = 'bmp'; 
if nargin<5, color_ext = 'jpeg'; end   % for 'stereoSeq', color_ext = 'bmp'; 
global debug_mode;

rgb_fname = dir([Folder_Name '/' RGB_FolderName '/*.' color_ext]);
[~,idx] = sortrows([rgb_fname.datenum].'); 
d_fname = dir([Folder_Name '/' D_FolderName '/*.' depth_ext]);

D = imread([Folder_Name '/' D_FolderName '/' d_fname(idx(1)).name]);  [H, W] = size(D);
color_BG = zeros(H, W); depth_BG = color_BG; cnt_map = color_BG;
% num_px = H*W;  mat_Y = zeros(num_px, 60);  mat_D = mat_Y;
for i=1:60  % use 60 frames for 'ColCamSeq'/ 'DCamSeq' / 'GenSeq' / 'ShSeq'
    % --------------------- color BG ---------------------
    I = imread([Folder_Name '/' RGB_FolderName '/' rgb_fname(idx(i)).name]);
    yuv_I = rgb2ycbcr(I);
    if debug_mode, figure(10), imshow(yuv_I(:,:,1)), title(num2str(i)), drawnow, end
    color_BG = (color_BG*(i-1) + double(yuv_I(:,:,1))) / i;
%     mat_Y(:,i) = reshape(double(yuv_I(:,:,1)), num_px, 1);
    % --------------------- depth BG --------------------- 
    D = imread([Folder_Name '/' D_FolderName '/' d_fname(idx(i)).name]);
%     mat_D(:,i) = reshape(double(D), num_px, 1);
    mask_D = D>0;
    cnt_map = cnt_map + double(mask_D);
    depth_BG(mask_D) = (depth_BG(mask_D).*(cnt_map(mask_D)-1) + double(D(mask_D))) ./ cnt_map(mask_D);
end
if debug_mode, figure(2), imshow(uint8(depth_BG/4096*255)), end
if debug_mode, figure(1), imshow(uint8(color_BG)), end
% [L_d, S_d] = SSGoDec(mat_D, 1, 30, 2, 100);   depth_BG = reshape(L_d(:,1), H, W);
% [L, S] = SSGoDec(mat_Y, 1, 5, 2, 100);        color_BG = reshape(L(:,1), H, W);