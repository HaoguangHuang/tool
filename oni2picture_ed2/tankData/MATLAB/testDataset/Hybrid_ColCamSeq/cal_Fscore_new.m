function res_vec = cal_Fscore_new(I_res, I_gt)
% [recall, precision, F_score] = cal_Fscore(I_res, I_gt)
% Quantitative evaluations with given ground truth
% res_vec = [TE, FNR, FPR, S];   % 1x4 row vector
% I_res is the result mask 
% I_gt is the ground truth

Res_nz = logical(I_res); Res_0 = ~Res_nz;
GT_nz =  logical(I_gt);  GT_0 = ~GT_nz;
[H, W] = size(I_res);
A_and_B = sum(sum(I_res & I_gt));    A_or_B = sum(sum(I_res | I_gt));

TP_map = Res_nz & GT_nz;   % True positive: pixels with correct 1s
TN_map = Res_0  & GT_0;    % True negative: pixels with correct 0s
FP_map = Res_nz & GT_0;    % False positive: pixels with incorrect 1s
FN_map = Res_0  & GT_nz;   % False negative: pixels with incorrect 0s
TP = sum(sum(TP_map)); TN = sum(sum(TN_map));
FP = sum(sum(FP_map)); FN = sum(sum(FN_map));

FPR = 100*FP/(FP+TN);                 % False Positive Rate
FNR = 100*FN/(TP+FN);                 % False Negative Rate
TE  = 100*(FP+FN)/(H*W);              % Total Error
 S  = A_and_B / A_or_B; 
res_vec = [TE, FNR, FPR, S];