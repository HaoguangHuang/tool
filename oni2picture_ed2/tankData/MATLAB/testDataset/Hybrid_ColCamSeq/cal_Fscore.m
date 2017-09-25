function res_vec = cal_Fscore(I_res, I_gt)
% [recall, precision, F_score] = cal_Fscore(I_res, I_gt)
% Quantitative evaluations with given ground truth
% res_vec = [TE, FNR, FPR, S];   % 1x8 row vector
% I_res is the result mask 
% I_gt is the ground truth

Res_nz = logical(I_res); Res_0 = ~Res_nz;
GT_nz =  logical(I_gt);  GT_0 = ~GT_nz;

TP_map = Res_nz & GT_nz;   % True positive: pixels with correct 1s
TN_map = Res_0  & GT_0;    % True negative: pixels with correct 0s
FP_map = Res_nz & GT_0;    % False positive: pixels with incorrect 1s
FN_map = Res_0  & GT_nz;   % False negative: pixels with incorrect 0s
TP = sum(sum(TP_map)); TN = sum(sum(TN_map));
FP = sum(sum(FP_map)); FN = sum(sum(FN_map));
Re = TP/(TP+FN);                  % Recall
Sp = TN/(TN+FP);                  % Specificity
FPR = FP/(FP+TN);                 % False Positive Rate
FNR = FN/(TP+FN);                 % False Negative Rate
PWC = 100*(FN+FP)/(TP+FN+FP+TN);  % Percentage of Wrong Classifications
Pr = TP/(TP+FP);                  % Precision
F_score = 2*Pr*Re/(Pr + Re);      % F-measure
JAC = TP/(FN + FP + TP);          % JAC index 
res_vec = [Re, Sp, FPR, FNR, PWC, Pr, F_score, JAC];