function [mask_gbf, weight_o, g_t]= guided_JBF(mask, I, is_ime, count_i, weight, ...
    sigma_c, win_width, thres)
    I = double(I);
    [H, W]  =size(mask);  mask_gbf = zeros(H,W);
    if nargin<6, sigma_c = 8;end   %the smaller the sigma_c is, and the more sensitive to color intensity difference
    if nargin<7,win_width = 11;end %奇数容易分配num
    if nargin<8,thres = 0.5; end   %[0,1]
    num_win = win_width*win_width;
    half_w = (win_width-1)/2;
    g_t = 0;                       %count the num of changing pixels in mask
    sigma_precompute = -2*sigma_c*sigma_c;
    r_start = half_w+1;            % round((win_height + 1)/2);  
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
                mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;%固定值，可以保存下来
                weight_o((r-1)*W+c,:) = weight_vec;
                res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                mask_gbf(r,c) = res_i;
            end
        end 
        return;
    end
    
   
    if is_ime == 1  %guided imerode
        for r = r_start : r_end
            for c = c_start : c_end          
                if mask(r,c)
                    mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                    
                    weight_vec = weight((r-1)*W+c,:);
                    
                    res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                    mask_gbf(r,c) = res_i;
                    
                end
            end
        end
    elseif is_ime == 0%guided imdilate
        for r = r_start : r_end
            for c = c_start : c_end          
                if ~mask(r,c)
                    mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                   
                    weight_vec = weight((r-1)*W+c,:);
                    
                    res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                    mask_gbf(r,c) = res_i;
                end
            end
        end
        mask_gbf = (mask_gbf + mask)>0;
        
    else  %is_ime == -1    do all pixels
        for r = r_start : r_end
            for c = c_start : c_end          
                    mask_vec = reshape(mask(r+win_vec, c+win_vec), 1, num_win);
                   
                    weight_vec = weight((r-1)*W+c,:);
                    
                    res_i = sum(weight_vec.*mask_vec)/sum(weight_vec);
                    mask_gbf(r,c) = res_i; 
            end
        end
    end
mask_gbf = mask_gbf>thres;
Res_I(:,:,1) = mask_gbf*255;
Res_I(:,:,2) = I;
Res_I(:,:,3) = zeros(H,W);
global debug_mode;
if debug_mode, figure(4), imshow(uint8(Res_I)), title('after JBF'),drawnow; end;
Res_I(:,:,1) = mask*255;
if debug_mode, figure(3), imshow(uint8(Res_I)), title('before JBF'),drawnow; end;
g_t = abs(sum(sum(logical(mask) - logical(mask_gbf))));
end
