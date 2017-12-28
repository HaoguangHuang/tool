%% IRP_ANALYSIS

function DoF_node_relation_map = IRP_analysis(pc1, pc2, corrIndex, rate, DoF_node_relation_map)
    if isempty(DoF_node_relation_map) % first and second frame
        DoF_node_relation_map = estimate_mode(pc1, pc2, corrIndex, rate, DoF_node_relation_map);
    else 
        
    end


end

function DoF_node_relation_map = estimate_mode(pc1, pc2, corrIndex, rate, DoF_node_relation_map)
    num_samples = 50; % num of choosed unique_corr pair 
    num_trials = 20;
    Pmat1 = pc1.Location; Pmat2 = pc2.Location;

    idx_uniq = corrIndex(:,1)>0 & corrIndex(:,2)>0;
    num_corr = sum(idx_uniq);
    uniq_corr = corrIndex(idx_uniq, :);
    Pmat1_uniq = Pmat1(uniq_corr(:,1), :);
    Pmat2_uniq = Pmat2(uniq_corr(:,2), :);
    thr_pose_inlier = 1;
    DoF_mat = [];
    
    % Initialization
    pose_i.R = eye(3); pose_i.T = zeros(1,3);
%     
%     stat(1).MAX_num_inlier = 0;
%     stat(1).num_trial = 1;
%     stat(1).pose = pose_i;
%     stat(1).inler_idx = true(1, num_corr);
    
    vec_num_inlier = zeros(1, num_trials); %num_trial = 20
    vec_L2err = zeros(1, num_trials);
    mat_poseR = zeros(3, 3*num_trials);
    mat_poseT = zeros(3, num_trials);
    
    %=======process=======
    res_pts_num = inf;
    num_DoF = 1;
    pts_thres = rate * num_corr;
    iter_num = 1;
    while res_pts_num > pts_thres && iter_num < 70
        stat(num_DoF).MAX_num_inlier = 0;
        stat(num_DoF).num_trial = 1;
        stat(num_DoF).pose = pose_i;
        stat(num_DoF).inler_idx = true(1, num_corr);
        for i=1:num_trials
            rng(i); idx_trial = randperm( num_corr, num_samples); %num_sample=30
            pose_i = est_pose_for_subset(Pmat1_uniq, Pmat2_uniq, idx_trial);
            [L2err_i, in_idx_i, num_inlier_i] = test_pose(pose_i, Pmat1_uniq, Pmat2_uniq, thr_pose_inlier);
            vec_num_inlier(i) = num_inlier_i; vec_L2err(i) = L2err_i;
            mat_poseR(:, (i-1)*3+(1:3)) = pose_i.R;
            mat_poseT(:, i) = pose_i.T;
            if stat(num_DoF).MAX_num_inlier<num_inlier_i
                stat(num_DoF).MAX_num_inlier = num_inlier_i;
                stat(num_DoF).num_trial = i;
                stat(num_DoF).pose = pose_i;
                stat(num_DoF).inler_idx = in_idx_i;
            end
        end
        % have found the best pose that have maximum inliers 
        if stat(num_DoF).MAX_num_inlier < 100
            iter_num = iter_num + 1;
            disp(['MAX_num_inlier is too little, now iter_num = ', int2str(iter_num)]);
            continue; 
        end
        
        Pmat1_uniq = Pmat1_uniq(~stat(num_DoF).inler_idx,:);
        Pmat2_uniq = Pmat2_uniq(~stat(num_DoF).inler_idx,:);
        res_pts_num = sum(~stat(num_DoF).inler_idx);
        num_DoF = num_DoF + 1;
        num_corr = res_pts_num;
        iter_num = iter_num + 1;
        
        pose_i.R = eye(3); pose_i.T = zeros(1,3);
        disp(['res_pts_num = ',int2str(res_pts_num)]);
        disp(['iter_num = ', int2str(iter_num)]);
    end
end


function pose_i = est_pose_for_subset(P1mat, P2mat, idx_trial)
    % Find the pose for a subset of corresponding points described by idx_trial
    P1_sset = P1mat(idx_trial, :); P2_sset = P2mat(idx_trial, :);
    pose_i = est_pose(P1_sset, P2_sset);
end


function [mean_L2err, inlier_idx, num_inlier] = test_pose(pose, P1, P2, thr)
    n = size(P1, 1);
    P1_transformed = P1*pose.R + repmat(pose.T, n, 1);
    ErrMat = P2 - P1_transformed;
    ssd_P1P2 = sum(ErrMat.^2, 2);  % sum of squared distances;
    dist_P1P2 = sqrt(ssd_P1P2); % per-point distances
    mean_L2err = mean(dist_P1P2);% mean distance;
    inlier_idx = dist_P1P2<thr;
    num_inlier = sum(inlier_idx); 
end


% -------------- Rigid pose estimation ---------------
function pose = est_pose(P1, P2)
    % Find R and T so that P2 = P1*R + T;
    [P1_new, P1_mean] = centralize_pointset(P1);
    [P2_new, P2_mean] = centralize_pointset(P2);
    H = P1_new'*P2_new;
    [U,~,V] = svd(H);
    X = V*U';
    pose.R = X';
    pose.T = P2_mean - P1_mean*X'; %transformation for p1--->p2
    if det(X)<0, disp('Algorithm fail.'); end
end

function [P_new, P_mean] = centralize_pointset(P)
    % P is a Nx3 matrix
    n = size(P, 1);
    P_mean = mean(P);
    P_new = P - repmat(P_mean, n, 1);
end
