%% TRANSFORMATION_JBF
% use D1 to interpolate transformation of D2
% idx:record indice of points that don't have nearest neighbor unique_corr points
function [transformationMap2, idx]= transformation_JBF(D1,D2,transformationMap1, idx_map2)
    [H, W] = size(D1);
    sigma_c = 9;  win_width = 5;  %win_width must be odd  
    num_win = win_width * win_width;
    half_w = (win_width - 1) / 2;
    sigma_precompute = -2 * sigma_c * sigma_c;
    r_start = half_w + 1; r_end = round(H - half_w - 1);
    c_start = half_w + 1; c_end = round(W - half_w - 1);
    
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);
    win_vec = -half_w:half_w;
    transformationMap = transformationMap1;  %total. 6DoF
    transformationMap2 = ones(size(transformationMap1))*inf;
    mask = D2~=inf;
    
    idx = zeros(H*W,1); cnt = 0;
    %%only operate  
    for i = 1:6
        m = transformationMap1(:,:,i);  %get value from a 3 dimension array. Cost a lot of time
        cnt = 0;
        for r = r_start : r_end
            for c = c_start : c_end
                if mask(r,c) ~= 0 
                    depth_patch = reshape(D1(r+win_vec, c+win_vec), 1, num_win);
                    depth_i = ones(1, num_win)*D2(r,c);
                    depth_vec = exp((depth_i - depth_patch).^2/sigma_precompute);  %fix value
                    
                    weight_vec = G1_vec .* depth_vec;  %fix value
                    
                    mask_vec = reshape(m(r+win_vec, c+win_vec), 1, num_win);

                    index = mask_vec ~= inf;
                    res_i = sum(weight_vec(index).*mask_vec(index))/sum(weight_vec(index));
                    if isnan(res_i)
                        cnt = cnt + 1; idx(cnt,1) = idx_map2(r,c);
                        continue; 
                    end
                    transformationMap2(r,c,i) = res_i;
                    transformationMap(r,c,i) = res_i; %not good. Since it may occur occlusion between transformationMap1 and transformationMap2
                end
            end
        end
    end
   
    idx = idx(1:cnt,1);


end