function [mask_gbf, count_o, weight_o]= guided_JBF(mask, I, is_ime, count_i, weight)
    I = double(I);
    [H, W]  =size(mask);  mask_gbf = zeros(H,W);
    win_width = 5;%奇数容易分配num
    win_height = 5; num_win = win_height*win_height;
    half_w = (win_width-1)/2;
    sigma_c = 5;
    thres = 0.5;
    sigma_precompute = -2*sigma_c*sigma_c;
    r_start = half_w+1; % round((win_height + 1)/2);  
    r_end = round(H - half_w-1);
    c_start = half_w+1;     c_end = round(W - half_w-1);
    
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);  win_vec = -half_w:half_w;
    count_o = count_i + 1;
    weight_o = zeros([H*W,num_win]);
    
    if count_i == 0 %
        for r = r_start : r_end
            for c = c_start : c_end          
                vec_patch = reshape(I(r+win_vec, c+win_vec), 1, num_win);
                vec_i = ones(1, num_win)*I(r,c);
                mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;%固定值，可以保存下来
                weight_o((r-1)*W+c,:) = weight_vec;
                res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                mask_gbf(r,c) = res_i;
            end
        end
        figure(1), imshow(mat2gray(mask_gbf), 'border', 'tight'), title('grayscale result')
        mask_gbf = mask_gbf>thres;
        Res_I(:,:,1) = mask_gbf*255;
        Res_I(:,:,2) = I;
        Res_I(:,:,3) = zeros(H,W);
        figure(4), imshow(uint8(Res_I), 'border', 'tight'), title('after JBF')
        % figure(2), imshow(mask_gbf, 'border', 'tight'), title('after JBF')
        Res_I(:,:,1) = mask*255;
        figure(3), imshow(uint8(Res_I), 'border', 'tight'), title('before JBF')
        
        return;
    end
    
    
    
    
    % G2_mat = zeros(H, W, num_win);
    if is_ime == 1  %guided imerode
        for r = r_start : r_end
            for c = c_start : c_end          
                if mask(r,c)
                    vec_patch = reshape(I(r+win_vec, c+win_vec), 1, num_win);
                    vec_i = ones(1, num_win)*I(r,c);
                    mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                    if count_i <= 1
                        weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;%固定值，可以保存下来
                        weight_o((r-1)*W+c,:) = weight_vec;
                    else 
                        weight_vec = weight((r-1)*W+c,:);
                    end
                    res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                    mask_gbf(r,c) = res_i;
    %                 figure(101), imshow(mat2gray(reshape(weight_vec, win_width, win_width))), title('weight')
    %                 figure(102), imshow(mat2gray(reshape(mask_vec, win_width, win_width))), title('mask\_in')
    %                 figure(103), imshow(uint8(reshape(vec_patch, win_width, win_width))), title('Image')
    %                 figure(100),  imshow(uint8(I)); hold on, rectangle('Position', [c-half_w, r-half_w, win_height, win_height], 'Edgecolor', 'r')
                end
            end
        end
    else %guided imdilate
        for r = r_start : r_end
            for c = c_start : c_end          
                if ~mask(r,c)
                    vec_patch = reshape(I(r+win_vec, c+win_vec), 1, num_win);
                    vec_i = ones(1, num_win)*I(r,c);
                    mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                   if count_i <= 1
                        weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;%固定值，可以保存下来
                        weight_o((r-1)*W+c,:) = weight_vec;
                    else %count>1
                        weight_vec = weight((r-1)*W+c,:);
                    end
                    res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                    mask_gbf(r,c) = res_i;
                end
            end
        end
        mask_gbf = (mask_gbf + mask)>0;
    end
figure(1), imshow(mat2gray(mask_gbf), 'border', 'tight'), title('grayscale result')
mask_gbf = mask_gbf>thres;
Res_I(:,:,1) = mask_gbf*255;
Res_I(:,:,2) = I;
Res_I(:,:,3) = zeros(H,W);
figure(4), imshow(uint8(Res_I), 'border', 'tight'), title('after JBF')
% figure(2), imshow(mask_gbf, 'border', 'tight'), title('after JBF')
Res_I(:,:,1) = mask*255;
figure(3), imshow(uint8(Res_I), 'border', 'tight'), title('before JBF')
  

end
