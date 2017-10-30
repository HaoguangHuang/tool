%优化版本的guided_JBF
% ps:相比原来的guided_JBF，预先计算了weight_s，来提高apply filter的速度
function [mask_gbf, weight_o, g_t, weight_s]= guided_JBF_opt(mask, I, is_ime, count_i, weight, ...
    sigma_c, win_width, thres, weight_sum)
    I = double(I);
    [H, W]  =size(mask);  
%     mask_gbf = zeros(H,W);
    mask_gbf = mask;
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
%     G1_vec = reshape(mat2gray(G1_mat), 1, num_win);  
    G1_vec = reshape(mat2gray(G1_mat), win_width, win_width);  
    
    win_vec = -half_w:half_w;

%     weight_o = zeros([H*W,num_win]);
%     weight_s = zeros([H*W,1]);
    weight_o = zeros([win_width, win_width, H*W]);
    weight_s = zeros([1,H*W]); %weight_sum
    %%only for count weight_o
    if count_i == 0 %
        t = zeros(1, H*W);
        for r = r_start : r_end
            for c = c_start : c_end          
                tic;
                vec_patch = I(r+win_vec, c+win_vec);
                vec_i = ones(win_width, win_width)*I(r,c);
                weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;%固定值，可以保存下来
                weight_o(:,:,(r-1)*W+c) = weight_vec;
                weight_s(:,(r-1)*W+c) = sum(sum(weight_o(:,:,(r-1)*W+c)));
                t(1,(r-1)*W+c) = toc;
            end
        end 
        disp(['Time of computing weight = ', num2str(sum(t))]);
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
%         count_zero = 0; count_one = 0; t = zeros(H*W,1);
        t = zeros(6,H*W); 
        for r = r_start : r_end
            for c = c_start : c_end          
                tic; mask_vec = mask_gbf(r+win_vec, c+win_vec); t(1,(r-1)*W+c) = toc;
                tic; if ~mask_vec, mask_gbf(r,c) = 0; continue; end; t(2,(r-1)*W+c) = toc;
                tic; if sum(sum(mask_vec))==num_win, mask_gbf(r,c) = 1; continue; end; t(3,(r-1)*W+c) = toc;
                tic; weight_vec = weight(:,:,(r-1)*W+c); t(4,(r-1)*W+c) = toc;
                tic; res_i = sum(sum(weight_vec.*mask_vec))/weight_sum((r-1)*W+c); t(5,(r-1)*W+c) = toc;
                tic; mask_gbf(r,c) = res_i>thres; t(6,(r-1)*W+c) = toc;    
            end
        end 
        disp(['t1 = ',num2str(sum(t(1,:)))]);
        disp(['t2 = ',num2str(sum(t(2,:)))]);
        disp(['t3 = ',num2str(sum(t(3,:)))]);
        disp(['t4 = ',num2str(sum(t(4,:)))]);
        disp(['t5 = ',num2str(sum(t(5,:)))]);
        disp(['t6 = ',num2str(sum(t(6,:)))]);

%         disp(['reshape time = ',num2str(sum(t))]);
%         disp(['count_zero = ',int2str(count_zero)]);
%         disp(['count_one = ',int2str(count_one)]);
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
