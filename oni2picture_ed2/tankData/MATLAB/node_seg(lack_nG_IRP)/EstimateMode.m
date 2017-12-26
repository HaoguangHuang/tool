function paras = EstimateMode(Pmat1, Pmat2, Corres)
% Estimate the independent parameters of an object from two 3D point clouds with correspondences
% Input:
%   Pmat# = [x1, y1, z1; x2, y2, z2; ...], where Pmat1 is a Nx3 matrix, Pmat2 is a Mx3 matrix
%   Corres = [i1, j1; i2, j2], where i belongs [1, N] and j belongs [1, M]
% Output parameters are in the form of:
%   paras(1).R, paras(1).T, paras(1).idx
%   paras(2).R, paras(2).T, paras(2).idx
%   ...
%   paras(k).R, paras(k).T, paras(k).idx
% where R and T are the rigid transforms, idx includes the indices of the
% points that belong to the "parameter subset".
% 
% Note: 
% - "parameter subset" is a set of points with the same rigid transform
% 2017-10-16: only for the case of k = 2;

num_DOF = 2;  % the number of free parameters/rigid parts
if nargin<1
    load point_data
    Pmat1 = pc1.Location;     ColorVec1 = pc1.Color(:,1); clear pc1
    Pmat2 = pc2.Location;    %  ColorMat2 = pc1.Color; 
    clear pc2
    Corres = corrIndex;    clear corrIndex
end

num_trial = 10*num_DOF;	% number of trials
num_samples = 30;       % number of samples
idx_uniq = Corres(:,1)>0 & Corres(:,2)>0;
num_corr = sum(idx_uniq);
uniq_corr = Corres(idx_uniq, :); clear Corres;
Pmat1_uniq = Pmat1(uniq_corr(:,1), :);   ColorVec1_uniq = ColorVec1(uniq_corr(:,1));
Pmat2_uniq = Pmat2(uniq_corr(:,2), :);   % ColorMat2_uniq = ColorMat2(uniq_corr(:,2), :);
thr_pose_inlier = 1;
% Produced a correspondence map
UVmat1 = round(proj_pcl(Pmat1_uniq, camera_para));
idx_map = zeros(480, 640); Y_map = zeros(480, 640);
for i=1:num_corr
    v = UVmat1(i,1);      u = UVmat1(i,2);
    idx_map(u, v) = i;    Y_map(u, v) = ColorVec1_uniq(i);
end
% figure(100), imshow(idx_map>0); title('pixels with unique correspondence')

% Initialization
pose_i.R = eye(3); pose_i.T = zeros(1,3);
for k=1:num_DOF
    stat(k).MAX_num_inlier = 0;
    stat(k).num_trial = 1;
    stat(k).pose = pose_i;
    stat(k).inler_idx = true(1, num_corr);
end
vec_num_inlier = zeros(1, num_trial); %num_trial = 20
vec_L2err = zeros(1, num_trial);
mat_poseR = zeros(3, 3*num_trial);
mat_poseT = zeros(3, num_trial);

% -------------------------- First round --------------------------
tic
for i=1:num_trial
    rng(i); idx_trial = randperm( num_corr, num_samples); %num_sample=30
    pose_i = est_pose_for_subset(Pmat1_uniq, Pmat2_uniq, idx_trial);
    [L2err_i, in_idx_i, num_inlier_i] = test_pose(pose_i, Pmat1_uniq, Pmat2_uniq, thr_pose_inlier);
    vec_num_inlier(i) = num_inlier_i; vec_L2err(i) = L2err_i;
    mat_poseR(:, (i-1)*3+(1:3)) = pose_i.R;
    mat_poseT(:, i) = pose_i.T;
    if stat(1).MAX_num_inlier<num_inlier_i
        stat(1).MAX_num_inlier = num_inlier_i;
        stat(1).num_trial = i;
%         stat(1).pose = pose_i;
        stat(1).inler_idx = in_idx_i;
    end
end
% have found the best pose that have maximum inliers 
idx_inlier_dense1 = extend_UVindex(Y_map, idx_map, UVmat1, uniq_corr(:,1), stat(1).inler_idx);  % 0.5 second
stat(1).inler_idx = idx_map(idx_inlier_dense1);
stat(1).pose = est_pose_for_subset(Pmat1_uniq, Pmat2_uniq, stat(1).inler_idx);
disp(num2str(toc,'Round 1: %.2e s'))

% ----------------------- debug info start -----------------------------
figure(1), clf, plot(vec_num_inlier/max(vec_num_inlier),'r'), hold on
plot(vec_L2err/max(vec_L2err),'b'), legend('vec\_num\_inlier','vec\_L2err'); title('Round 1')
figure(2), clf, plot3(Pmat1_uniq(:,1),Pmat1_uniq(:,2),Pmat1_uniq(:,3),'r.'); hold on;
plot3(Pmat1_uniq(stat(1).inler_idx,1),Pmat1_uniq(stat(1).inler_idx,2),Pmat1_uniq(stat(1).inler_idx,3),'bx');
legend('Original P1', 'Selected inliers'); title('Round 1')
disp(num2str([stat(1).num_trial stat(1).MAX_num_inlier],'%d-th trial: %d inliers'))
disp(stat(1).pose.R); disp(stat(1).pose.T)
% ----------------------- debug info end -----------------------------


% ------------------------- Second round ---------------------------------
idx_uniq2_map = idx_map(~idx_inlier_dense1);
idx_uniq2 = idx_uniq2_map(idx_uniq2_map>0);
num_corr2 = length(idx_uniq2);
Pmat1_uniq2 = Pmat1_uniq(idx_uniq2,:);
Pmat2_uniq2 = Pmat2_uniq(idx_uniq2,:);
% thr_pose_inlier = thr_pose_inlier;
% num_samples = num_samples/3;  
for i=1:num_trial
    rng(i); idx_trial = randperm( num_corr2, num_samples); 
    pose_i = est_pose_for_subset(Pmat1_uniq2, Pmat2_uniq2, idx_trial);
    [L2err_i, in_idx_i, num_inlier_i] = test_pose(pose_i, Pmat1_uniq2, Pmat2_uniq2, thr_pose_inlier);
    vec_num_inlier(i) = num_inlier_i; vec_L2err(i) = L2err_i;
    if stat(2).MAX_num_inlier<num_inlier_i
        stat(2).MAX_num_inlier = num_inlier_i;
        stat(2).num_trial = i;
        stat(2).pose = pose_i;
        stat(2).inler_idx = in_idx_i;
    end
end
stat(2).pose = est_pose_for_subset(Pmat1_uniq2, Pmat2_uniq2, stat(2).inler_idx);
idx_inlier_dense2 = extend_UVindex(Y_map, idx_map, UVmat1, uniq_corr(:,1), stat(2).inler_idx);  % 0.5 second
disp(num2str(toc, '\nRound 2: %.2e s'))
% ------------------- debug info ---------------------
figure(3), clf, plot(vec_num_inlier/max(vec_num_inlier),'r'), hold on
plot(vec_L2err/max(vec_L2err),'b'), legend('vec\_num\_inlier','vec\_L2err'); title('Round 2')
figure(4), clf, plot3(Pmat1_uniq2(:,1),Pmat1_uniq2(:,2),Pmat1_uniq2(:,3),'r.'); hold on;
plot3(Pmat1_uniq2(stat(2).inler_idx,1),Pmat1_uniq2(stat(2).inler_idx,2),Pmat1_uniq2(stat(2).inler_idx,3),'bx');
legend('Points before Round 2', 'Inliers after Round 2'); title('Round 2')
disp(num2str([stat(2).num_trial stat(2).MAX_num_inlier],'%d-th trial: %d inliers'))
disp(stat(2).pose.R); disp(stat(2).pose.T)
% -----------------------------------------------------
figure(5), imshow(idx_inlier_dense1); title('Rigid part 1')
figure(6), imshow(idx_inlier_dense2); title('Rigid part 2')


% ------------------------ rigid part estimation --------------------------
function new_inlier_mask = extend_UVindex(Y_map, idx_map, UVmat, corr_all, idx_inlier)
% new_idx = extend_UVindex(Y_map, idx_map, old_idx); 
% - new_idx is a dense indices, old_idx is a sparse one;
% This funtion updates the correspondence indices with given intensity map, 
%   index map, and original correspondence indices 
[H, W] = size(idx_map);
n = length(idx_inlier);
% create the sparse inlier mask
new_inlier_mask = false(H, W);
for i=1:n
    if idx_inlier(i)
        v = UVmat(i,1); u = UVmat(i,2);
        new_inlier_mask(u, v) = 1;
    end
end

% extend the inlier mask by BF weighted averaging
BF_half_win = 5;  sigma_c = 8;
win_idx = -BF_half_win:BF_half_win;
Gs = fspecial('gaussian', 2*BF_half_win+1, BF_half_win);   Gs_vec = Gs(:);
diff_vec = zeros(255, 1); for i=1:255, diff_vec(i) = exp(-(i-1)^2/(2*sigma_c*sigma_c)); end
for kk = 1:3
    for i=1:n
        v = UVmat(i,1); u = UVmat(i,2);
        if kk<3 || ~idx_inlier(i)
            Y_patch = Y_map(u+win_idx, v+win_idx);                  Y_p_vec = Y_patch(:);
            inlier_patch = new_inlier_mask(u+win_idx, v+win_idx);   inlier_p_vec = inlier_patch(:);
%             comp_idx = inlier_patch>0;                          comp_idx_vec = comp_idx(:);
            Y_diff_vec = abs(Y_p_vec - Y_map(u, v))+1;
        %     wt_vec =
        %     Gs_vec(comp_idx_vec).*diff_vec(Y_diff_vec(comp_idx_vec));         % BF weight(selected)
        %     new_inlier_mask(u, v) = sum(wt_vec.*inlier_p_vec(comp_idx_vec))/sum(wt_vec);  % weighted averaging
            wt_vec = Gs_vec.*diff_vec(Y_diff_vec);   % BF weight
            new_inlier_mask(u, v) = sum(wt_vec.*inlier_p_vec)/sum(wt_vec)>0.5; % weighted averaging
            if mod(i, 1000)==0
                figure(111), imshow(new_inlier_mask); title(num2str(i,'pixel %d')), drawnow()
            end
        end
    end
end 


function UVmat = proj_pcl(PCL, camera_para)
% find image coordinates by projection
K = [camera_para.fx 0 camera_para.cx; 0 camera_para.fy camera_para.cy; 0 0 1];
UV1 = PCL*K';
UVmat = UV1(:, 1:2)./[UV1(:,3) UV1(:,3)];


function pose_i = est_pose_for_subset(P1mat, P2mat, idx_trial)
% Find the pose for a subset of corresponding points described by idx_trial
P1_sset = P1mat(idx_trial, :); P2_sset = P2mat(idx_trial, :);
pose_i = est_pose(P1_sset, P2_sset);

function [mean_L2err, inlier_idx, num_inlier] = test_pose(pose, P1, P2, thr)
n = size(P1, 1);
P1_transformed = P1*pose.R + repmat(pose.T, n, 1);
ErrMat = P2 - P1_transformed;
ssd_P1P2 = sum(ErrMat.^2, 2);  % sum of squared distances;
dist_P1P2 = sqrt(ssd_P1P2); % per-point distances
mean_L2err = mean(dist_P1P2);% mean distance;
inlier_idx = dist_P1P2<thr;
if nargout>2, num_inlier = sum(inlier_idx); end

% -------------- Rigid pose estimation ---------------
function pose = est_pose(P1, P2)
% Least-Squares Fitting of Two 3-D Point Sets, PAMI 1987.
% Find R and T so that P2 = P1*R + T;
[P1_new, P1_mean] = centralize_pointset(P1);
[P2_new, P2_mean] = centralize_pointset(P2);
H = P1_new'*P2_new;
[U,~,V] = svd(H);
X = V*U';
pose.R = X';
pose.T = P2_mean - P1_mean*X'; %transformation for p1--->p2
if det(X)<0, disp('Algorithm fail.'); end
 
function [P_new, P_mean] = centralize_pointset(P)
% P is a Nx3 matrix
n = size(P, 1);
P_mean = mean(P);
P_new = P - repmat(P_mean, n, 1);