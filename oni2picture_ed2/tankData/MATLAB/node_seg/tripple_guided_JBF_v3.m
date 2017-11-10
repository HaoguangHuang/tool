%% TRIPPLE_GUIDED_JBF
%  Interpolate points in D3 that have depth value but have no unique_corr
%  D1:480*640.                 projected uvd from pc1 that have unique correspondence.   guidance
%  D3:480*640.                 projected uvd from all points in pc2.                     operated mask
%  Y1:480*640.                 projected uvY from pc1 that have unique correspondence.
%  Y3:480*640.                 projected uvY from all points in pc2projected uvd from all points in pc2
%mask:480*640.                 pixel set that needed to be interpolated to get transformation
%transformationMap:480*640*6.  transformation of all points from pc2
%transformationMap2:480*640*6  transformation of points that have unique_corr from pc2
%transformationMap1:480*640*6  transformation of points that have unique_corr from pc1
function transformationMap = tripple_guided_JBF_v3(D1, D3, Y1, Y3, mask, transformationMap1, transformationMap2)
    [H, W] = size(D3);
    sigma_c = 9;  win_width = 5;  %win_width must be odd  
    num_win = win_width * win_width;
    half_w = (win_width - 1) / 2;
    sigma_precompute = -2 * sigma_c * sigma_c;
    r_start = half_w + 1; r_end = round(H - half_w - 1);
    c_start = half_w + 1; c_end = round(W - half_w - 1);
    
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);
    win_vec = -half_w:half_w;
    addMap = zeros(H,W,6);  %record points obtained from interpolation
    transformationMap = transformationMap2;  %6DoF
    %%only operate  
    for i = 1:6
        m = transformationMap2(:,:,i);  %get value from a 3 dimension array. Cost a lot of time
        for r = r_start : r_end
            for c = c_start : c_end
                if mask(r,c) ~= 0 
                    depth_patch = reshape(D1(r+win_vec, c+win_vec), 1, num_win);
                    depth_i = ones(1, num_win)*D3(r,c);
                    depth_vec = exp((depth_i - depth_patch).^2/sigma_precompute);  %fix value

                    intensity_patch = reshape(Y1(r+win_vec, c+win_vec), 1, num_win);
                    intensity_i = ones(1, num_win)*Y3(r,c);
                    intensity_vec = exp((intensity_i - intensity_patch).^2/sigma_precompute);  %fix value

                    weight_vec = G1_vec .* depth_vec .* intensity_vec;  %fix value
                    
                    mask_vec = reshape(m(r+win_vec, c+win_vec), 1, num_win);

                    index = mask_vec ~= 0;
                    res_i = sum(weight_vec(index).*mask_vec(index))/sum(weight_vec(index));
                    if isnan(res_i), continue; end
                    transformationMap(r,c,i) = res_i;
                    addMap(r,c,i) = res_i;
                end
            end
        end
    end
   

end