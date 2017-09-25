%% GET_FILE_ID
%  getFileID : This file is used for get file ID in groundtruth file
%  gtFile    : groundTruth file address
%  idx       : array. size of N*1
function idx = getFileID(gtFile)
    gt_fname = dir(gtFile);
    fileID   = size(gt_fname,1)-3;  %exclude '.', '..', and border file
    idx      = zeros(fileID, 1);
    for i = 3:size(gt_fname,1)-1
        n = gt_fname(i).name;     %name
        Start = 4; End = strfind(n,'B')-1;
        id = n(Start:End);
        idx(i-2) = str2double(id);
    end
    idx = sort(idx);
end