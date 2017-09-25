%% TRANSFORM_GROUNDTRUTH
% Since gt in GSM is three channels(uint8*3). Now we changes its format into logical, and name its suffix 
%   as '.bmp'
clear all;
root_dir = 'E:\dataSet\ICAISS\GSM\GSM_dataset\GSM\TimeOfDay_ds\groundTruth\';
gt_fname = dir([root_dir,'*.png']);
[~,idx] = sortrows([gt_fname.datenum]');   %get the file number
for i = 1:size(idx,1)
    gt = imread([root_dir,gt_fname(idx(i)).name]);
    %%======transform gt from format uint8*3 into logical=======
    res = logical(gt(:,:,1)>0);   
    imwrite(res,[root_dir,gt_fname(idx(i)).name(1:end-4),'.bmp']);
end


