function [recvy_d, weight_o, g_t]= guided_d_recvy(d, I, is_ime, count_i, weight, m)
    I = double(I);
    d = double(d);
    [H, W]  =size(d);  recvy_d = zeros(H,W);
    win_width = 11;%奇数容易分配num
    win_height =11; num_win = win_height*win_height;
    half_w = (win_width-1)/2;
    sigma_c = 8;
    thres = 0;
    g_t = 0;%count the num of changing pixels in mask
    sigma_precompute = -2*sigma_c*sigma_c;
    r_start = half_w+1; % round((win_height + 1)/2);  
    r_end = round(H - half_w-1);
    c_start = half_w+1;     c_end = round(W - half_w-1);
    
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);  win_vec = -half_w:half_w;

    weight_o = zeros([H*W,num_win]);
    
    %%only for count weight_o
    if count_i == 0 %
        for r = r_start : r_end
            for c = c_start : c_end          
                vec_patch = reshape(I(r+win_vec, c+win_vec), 1, num_win);
                vec_i = ones(1, num_win)*I(r,c);
                d_vec = reshape(d(r+win_vec, c+win_vec), 1, num_win);
                weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;%固定值，可以保存下来
                weight_o((r-1)*W+c,:) = weight_vec;
            end
        end 
        return;
    end
    
   
    if is_ime == 1  %guided imerode
        for r = r_start : r_end
            for c = c_start : c_end          
                if d(r,c)
                    d_vec = reshape(d(r+win_vec, c+win_vec), 1, num_win);
                    
                    weight_vec = weight((r-1)*W+c,:);
                    
                    res_i = sum(weight_vec.*d_vec)/sum(weight_vec);
                    recvy_d(r,c) = res_i;
                    
                end
            end
        end
    elseif is_ime == 0%guided imdilate
        for r = r_start : r_end
            for c = c_start : c_end          
                if ~d(r,c)
                    d_vec = reshape(d(r+win_vec, c+win_vec), 1, num_win);
                   
                    weight_vec = weight((r-1)*W+c,:);
                    
                    res_i = sum(weight_vec.*d_vec)/sum(weight_vec);
                    recvy_d(r,c) = res_i;
                end
            end
        end
        recvy_d = recvy_d + d;
        
    else  %is_ime == -1    do all pixels
        for r = r_start : r_end
            for c = c_start : c_end 
                %%only process pixel in mask
                if m(r,c) ~=0 && d(r,c) == 0
                    d_vec = reshape(d(r+win_vec, c+win_vec), 1, num_win);
                   
                    weight_vec = weight((r-1)*W+c,:);
                    
                     
%                     if isempty(d_m)
%                         res_i = 0;
%                     else
%                         res_i = sum(weight_vec(d_m).*d_vec(d_m))/sum(weight_vec(d_m));
%                     end
                    d_m = d_vec >0;
                     s = sum(weight_vec .*d_m);
                     if s == 0   % /s will get NaN
                         res_i = 0;
                     else
                        res_i = sum(weight_vec(d_m).*d_vec(d_m))/s;
                     end
                    recvy_d(r,c) = res_i; 
                end
            end
        end
        zengliang = recvy_d;
        t = zengliang(zengliang >0);
        recvy_d = recvy_d + d;
    end
% figure(1), imshow(mat2gray(mask_gbf), 'border', 'tight'), title('grayscale result')
recvy_d = recvy_d;
Res_I(:,:,1) = recvy_d*255;
Res_I(:,:,2) = I(:,:,1);
Res_I(:,:,3) = zeros(H,W);
figure(4), imshow(uint8(Res_I), 'border', 'tight'), title('after JBF')
% figure(2), imshow(mask_gbf, 'border', 'tight'), title('after JBF')
Res_I(:,:,1) = (d>0)*255;
figure(3), imshow(uint8(Res_I), 'border', 'tight'), title('before JBF')
  
Res_I(:,:,1) = zengliang*255;
figure(16), imshow(uint8(Res_I), 'border', 'tight'), title('zengliang')
Res_I(:,:,1) = m*255;
figure(17), imshow(uint8(Res_I), 'border', 'tight'), title('mask')
g_t = abs(sum(sum(logical(d) - logical(recvy_d))));


end
