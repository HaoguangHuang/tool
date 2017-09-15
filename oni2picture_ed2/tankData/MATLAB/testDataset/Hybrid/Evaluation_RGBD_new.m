function [mean_res, res_mat] = Evaluation_RGBD_new(Folder_Name, Res_FolderName, ResExt, GT_FolderName)
% [mean_res, res_mat] = Evaluation_RGBD(Folder_Name, Res_FolderName, ResExt, GT_FolderName)
% Run this file in the root dir of the RGB-D dataset to evaluate FG segmentation methods:
%   mean_res includes the mean value of [TE, FNR, FPR, S]
%   e.g.: mean_res = Evaluation_RGBD_new('ColCamSeq', 'res', 'png');
% - Folder_Name can be 'ColCamSeq'/ 'DCamSeq' / 'GenSeq' / 'movedBG' / 'ShSeq'
% * Note: for 'ColCamSeq' dataset, "depth_934.png" does not exist;

if nargin<1, Folder_Name = 'ShSeq'; end
if nargin<2, Res_FolderName = 'depthData'; end
if nargin<3, ResExt = 'png'; end
if nargin<4, GT_FolderName = 'groundTruth'; end
if nargin<5, GT_ext = 'bmp'; end
% debug_mode = 1;   

GT_fname = dir([Folder_Name '/' GT_FolderName '/*.' GT_ext]);
[~,idx] = sortrows([GT_fname.datenum].');  num_GT = length(idx);

D = imread([Folder_Name '/' GT_FolderName '/' GT_fname(idx(1)).name]);
[H, W] = size(D);  num_px = H*W;
res_mat = zeros(num_GT, 4);  % 4D detection accuracy
for i=1:num_GT  % 'ColCamSeq'/ 'DCamSeq' / 'GenSeq' / 'ShSeq'
    str_gtname = GT_fname(idx(i)).name;
    frame_no = sscanf(str_gtname, 'gt_%d.bmp');
    GT_mask = imread([Folder_Name '/' GT_FolderName '/' GT_fname(idx(i)).name]);
    % Note: results should be stored using the name like "res_%d.png" or "res_%d.bmp"
    Res_mask = imread([Folder_Name '/' Res_FolderName '/res_' num2str(frame_no) '.' ResExt]);
    res_vec = cal_Fscore_new(double(Res_mask), double(GT_mask));
    res_mat(i,:) = res_vec;
end
mean_res = mean(res_mat); std_res = std(res_mat);



