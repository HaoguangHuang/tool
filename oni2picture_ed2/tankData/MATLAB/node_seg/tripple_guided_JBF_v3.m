%% TRIPPLE_GUIDED_JBF
%  插值出D3深度图中有深度值但不拥有
%  D1:480*640.                 pc1中拥有unique correspondence的点的2D投影图uvd.   guidance
%  D3:480*640.                 pc2的所有点的2D投影图uvd.                          被操作的mask
%  Y1:480*640.                 pc1中拥有unique correspondence的点的2D投影图uvY.
%  Y3:480*640.                 pc2的所有点的2D投影图uvY
%mask:480*640.                 需要被插值出transformation的像素点集合
%transformationMap:480*640*3.  pc2中所有点的投影的transformation。
%transformationMap2:480*640*3  pc2中拥有unique_corr的点的投影的transformation
%transformationMap1:480*640*3  pc1中拥有unique_corr的点的投影的transformation
function transformationMap = tripple_guided_JBF_v3(D1, D3, Y1, Y3, mask, transformationMap1, transformationMap2)
    [H, W] = size(D3);
    sigma_c = 9;  win_width = 5;  %win_width必须是奇数  
    num_win = win_width * win_width;
    half_w = (win_width - 1) / 2;
    sigma_precompute = -2 * sigma_c * sigma_c;
    r_start = half_w + 1; r_end = round(H - half_w - 1);
    c_start = half_w + 1; c_end = round(W - half_w - 1);
    
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);
    win_vec = -half_w:half_w;
    addMap = zeros(H,W,6);  %记录通过插值得到的点
    transformationMap = transformationMap2;  %6DoF
    %%只操作mask中的点 
    for i = 1:6
        m = transformationMap2(:,:,i);  %三维数据的取值，速度很慢
        for r = r_start : r_end
            for c = c_start : c_end
                if mask(r,c) ~= 0 
                    depth_patch = reshape(D1(r+win_vec, c+win_vec), 1, num_win);
                    depth_i = ones(1, num_win)*D3(r,c);
                    depth_vec = exp((depth_i - depth_patch).^2/sigma_precompute);%固定值，可以保存下来

                    intensity_patch = reshape(Y1(r+win_vec, c+win_vec), 1, num_win);
                    intensity_i = ones(1, num_win)*Y3(r,c);
                    intensity_vec = exp((intensity_i - intensity_patch).^2/sigma_precompute);%固定值，可以保存下来

                    weight_vec = G1_vec .* depth_vec .* intensity_vec;%固定值，可以保存下来
                    
                    mask_vec = reshape(m(r+win_vec, c+win_vec), 1, num_win);

                    index = mask_vec ~= 0;
                    res_i = sum(weight_vec(index).*mask_vec(index))/sum(weight_vec(index));
                    if isnan(res_i), continue; end;
                    transformationMap(r,c,i) = res_i;
                    addMap(r,c,i) = res_i;
                end
            end
        end
    end
   

end