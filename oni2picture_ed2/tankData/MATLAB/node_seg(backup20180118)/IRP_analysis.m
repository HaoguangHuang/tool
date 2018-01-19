%% IRP_ANALYSIS

function DoF_node_relation_map = IRP_analysis(pc1, pc2, corrIndex, rate, DoF_node_relation_map)
    if isempty(DoF_node_relation_map) % first and second frame
        DoF_node_relation_map = estimate_mode(pc1, pc2, corrIndex, rate, DoF_node_relation_map);
    else 
        
    end


end

function DoF_node_relation_map = estimate_mode(pc1, pc2, corrIndex, rate, DoF_node_relation_map)
    num_samples = 300; % num of choosed unique_corr pair 
    num_trials = 20;
    Pmat1 = pc1.Location; Pmat2 = pc2.Location;

    idx_uniq = corrIndex(:,1)>0 & corrIndex(:,2)>0;
    num_corr = sum(idx_uniq);
    uniq_corr = corrIndex(idx_uniq, :);
    Pmat1_uniq = Pmat1(uniq_corr(:,1), :);
    Pmat2_uniq = Pmat2(uniq_corr(:,2), :);
    thr_pose_inlier = 2;
    DoF_mat = [];
    stat = [];
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
%             rng(i);
%             idx_trial = randperm(num_corr, num_samples); %num_sample=30
            
            % sample a pts set in a radius 
            core_idx = randperm(num_corr, 1);
            core_pt = Pmat1_uniq(core_idx,:);
            [idx_trial, ~] = findNearestNeighbors(pointCloud(Pmat1_uniq),core_pt, num_samples);
            idx_trial = idx_trial';
            
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
        
        %%======visualize inlier of  DoF======
        pc_inlier = Pmat1_uniq(stat(num_DoF).inler_idx,:);
        pc_inlier = pointCloud(pc_inlier);
        pc_inlier.Color = repmat(uint8([255,0,0]),pc_inlier.Count,1);
        figure(num_DoF),pcshow(pc1),hold on, pcshow(pc_inlier);
        title(sprintf('MAX_inlier of DoF %d, total pts = %d',num_DoF,stat(num_DoF).MAX_num_inlier));
        hold off;
        
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
    
    visualize_DoF(stat);
    
end

function visualize_DoF(stat)
    MarkerSpecs = {'d','o','x','+'};
    ColorSpecs = {'r','g','b','k'};
    
    n = size(stat, 2);
    %===visualize rotation===
%     close 55;
    figure(55); hold on;
    for i = 1:n
        yushu = mod(i,4); shang = floor(i/4);
        R = stat(i).pose.R;
        r = rodrigues(R);
        plot3(r(1),r(2),r(3),[ColorSpecs{shang+1},MarkerSpecs{yushu+1}],'markersize',10,'DisplayName',int2str(i)); 
    end
%     set(gca,'xtick',[-0.6:0.04:0.4]);
%     set(gca,'ytick',[-0.8:0.04:4]);
%     set(gca,'ztick',[-0.25:0.04:-0.5]);

%     axis([-0.2,0.2,-0.2,0.2,-0.5,0.5]);
    axis square;
    legend('show');
    title('rotation'); 
    xlabel('x'),ylabel('y'),zlabel('z'); 
    hold off;
    %===visualize translation===
%     close 66;
    figure(66); hold on;
    for i = 1:n
        yushu = mod(i,4); shang = floor(i/4);
        t = stat(i).pose.T;
        plot3(t(1),t(2),t(3),[ColorSpecs{shang+1},MarkerSpecs{yushu+1}],'markersize',10,'DisplayName',int2str(i));    
    end
    legend('show');
    title('translation');
    xlabel('x'),ylabel('y'),zlabel('z');
%     axis([-60,30,-40,60,-50,50]);
    axis square;
    hold off;
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
