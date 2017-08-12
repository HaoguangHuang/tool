function mask_gbf = guided_JBF(mask, I)
    I = double(I);
    [H, W]  =size(mask);  mask_gbf = zeros(H,W);
    win_width = 15;%奇数容易分配num
    win_height = 15; num_win = win_height*win_height;
    half_w = (win_width-1)/2;
    sigma_c = 7;
    thres = 0.5;
%     window = ones(win_height,win_width);
    sigma_precompute = -2*sigma_c*sigma_c;
    r_start = half_w+1; % round((win_height + 1)/2);  
    r_end = round(H - half_w-1);
    c_start = half_w+1;     c_end = round(W - half_w-1);
%    num =(r_end - r_start + 1)*(c_end - c_start + 1);%待处理的pixel数目
    %预分配空间
    % G1_array = zeros(size(window));%存放 (i - j)^2 
    
    %%计算G1_array以及相对偏移矩阵deviate_array
%     [G1_array, deviate_array_r, deviate_array_c] = precomputeArray(win_width, win_height);
%     G1_array = exp(G1_array ./ sigma_precompute);  G1_array = G1_array/norm(G1_array(:));
%     G1_vec = reshape(G1_array, 1, num_win);
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);  win_vec = -half_w:half_w;
    % G2_mat = zeros(H, W, num_win);
     figure(100), 
    for r = r_start : r_end
        for c = c_start : c_end          
            % G2_mat(r,c,:)
%             if ~mask(r,c)
                vec_patch = reshape(I(r+win_vec, c+win_vec), 1, num_win);
                vec_i = ones(1, num_win)*I(r,c);
                mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;
                res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                mask_gbf(r,c) = res_i;
%                 figure(101), imshow(mat2gray(reshape(weight_vec, win_width, win_width))), title('weight')
%                 figure(102), imshow(mat2gray(reshape(mask_vec, win_width, win_width))), title('mask\_in')
%                 figure(103), imshow(uint8(reshape(vec_patch, win_width, win_width))), title('Image')
%                 figure(100),  imshow(uint8(I)); hold on, rectangle('Position', [c-half_w, r-half_w, win_height, win_height], 'Edgecolor', 'r')
%             end
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
  
%     G2_array(size(mask,1) * size(mask,2)).data = zeros(size(window));%存放 (y_i - y_j)^2
%     G_array(size(mask,1) * size(mask,2)).data = zeros(size(window));%G = G1 .* G2
% %     deviate_array_r = zeros(size(G1_array));%纵向相对偏移矩阵
% %     deviate_array_c = zeros(size(G1_array));%横向相对偏移矩阵
%     mask_gbf = zeros(size(mask));
%     
%     
%     %%计算G2_array, 顺便也计算G = G1 .* G2
%     for r = r_start : r_end
%         for c = c_start : c_end
%             
%             
%             
%             G2_array(r*W+c).data = computeIntensity(I, r, c, win_width, win_height,...
%                 deviate_array_r, deviate_array_c);
%             %%计算G
%             G2_array(r*W+c).data = exp(G2_array(r*W+c).data ./ sigma_precompute);
%             G_array(r*W+c).data = G1_array .* G2_array(r*W+c).data;
%             
%             %%归一化G
% %             G_min = min(min(G_array(r*col+c).data));
% %             G_max = max(max(G_array(r*col+c).data));
% %             G_array(r*col+c).data = (G_array(r*col+c).data - G_min) ./ (G_max - G_min);
%                 G_array(r*W+c).data = G_array(r*W+c).data ./ sum(sum(G_array(r*W+c).data));
%             
%             %%取出当前像素(r,c)对应的windows中每个像素的真实位置
%             origin = [r,c];
%             mask_window = zeros(size(window));%取出(r,c)在mask中对应的窗口
%             for i = 1:win_height
%                 for j = win_width
%                      index = [origin(1) + deviate_array_r(i, j), origin(2) + deviate_array_c(i, j)];
%                      mask_window(i,j) = mask(index(1),index(2));
%                 end
%             end
%             
%             %%mask_gbf_i = sum(α_j * G_i), j为i的相邻像素
%             mask_gbf(r,c) = sum(sum(mask_window .* G_array(r*W+c).data));
%         end
%     end
% %     
%     mask_gbf = mask_gbf > thres;
%     
end


function [g1, d_array_r, d_array_c] = precomputeArray(windows_width, windows_height)
    g1 = zeros(windows_height, windows_width);
    d_array_r = zeros(windows_height, windows_width);
    d_array_c = zeros(windows_height, windows_width);
    origin = [(windows_height+1)/2, (windows_width+1)/2];
    for r = 1:windows_height
        for c = 1:windows_width
            g1(r,c) = (r - origin(1))^2 + (c - origin(2))^2;
            d_array_r(r,c) = r - origin(1);
            d_array_c(r,c) = c - origin(2);
        end
    end
end

function g2 = computeIntensity(fusionedForegroundData, r, c, windows_width, windows_height,...
    d_array_r, d_array_c)
    g2 = zeros(windows_height, windows_width);    
    origin = [r, c];
    for w_h = 1:windows_height
        for w_w = 1:windows_width
            index = [origin(1) + d_array_r(w_h, w_w), origin(2) + d_array_c(w_h, w_w)];%根据相对偏移表，获得windows中每个像素点的真实下标
            g2(w_h, w_w) = (fusionedForegroundData(index(1),index(2)) - fusionedForegroundData(r,c))^2;%返回一个深度值
        end
    end
end