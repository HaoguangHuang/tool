%% TRIPPLE_GUIDED_JBF
%  D1:480*640. pc1��ӵ��unique correspondence�ĵ��2DͶӰͼuvd.   guidance
%  D2:480*640. pc2��2DͶӰͼuvd.                                 ��������mask
%  Y1:480*640. pc1��ӵ��unique correspondence�ĵ��2DͶӰͼuvY.
%  Y2:480*640. pc2�ĵ��2DͶӰͼuvY
%mask:480*640. ��Ҫ����ֵ��pc1��corr�����ص㼯��
%mask_idx_map: 480*640. ÿ�����ص��ֵΪ�õ��Ӧpc1��corr��index
%addMap_idx_map:D2�в�ֵ������corr���pc1
%  Note: ע��õ���dense map��ֻ����uvd������Ϣ����������intensity
function [denseMap2, addMap_idx_map] = tripple_guided_JBF(D1, D2, Y1, Y2, mask, mask_idx_map)
    [H, W] = size(D2);
    denseMap2 = D2;
    sigma_c = 8;  win_width = 7;  %win_width����������  
    num_win = win_width * win_width;
    half_w = (win_width - 1) / 2;
    sigma_precompute = -2 * sigma_c * sigma_c;
    r_start = half_w + 1; r_end = round(H - half_w - 1);
    c_start = half_w + 1; c_end = round(W - half_w - 1);
    
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);
    win_vec = -half_w:half_w;
    addMap = zeros(H,W);  %��¼ͨ����ֵ�õ��ĵ�
    addMap_idx_map = zeros(H,W);
    %%ֻ����mask�еĵ�
    count = 0;
    for r = r_start : r_end
        for c = c_start : c_end
            if mask(r,c) ~= 0 
            depth_patch = reshape(D1(r+win_vec, c+win_vec), 1, num_win);
            depth_i = ones(1, num_win)*D2(r,c);
            depth_vec = exp((depth_i - depth_patch).^2/sigma_precompute);%�̶�ֵ�����Ա�������
            
            intensity_patch = reshape(Y1(r+win_vec, c+win_vec), 1, num_win);
            intensity_i = ones(1, num_win)*Y2(r,c);
            intensity_vec = exp((intensity_i - intensity_patch).^2/sigma_precompute);%�̶�ֵ�����Ա�������
            
            weight_vec = G1_vec .* depth_vec .* intensity_vec;%�̶�ֵ�����Ա�������
            
            mask_vec = reshape(D2(r+win_vec, c+win_vec), 1, num_win);
            
            index = mask_vec > 0;
            res_i = sum(weight_vec(index).*mask_vec(index))/sum(weight_vec(index));
            if isnan(res_i), continue; end;
            addMap_idx_map(r,c) = mask_idx_map(r,c);
            denseMap2(r,c) = res_i;
            count = count + 1; 
            addMap(r,c) = res_i;
            end
        end
    end
    
    %%thres������ã�
    I = zeros(H,W,3);
    I(:,:,1) = mat2gray(D2)*255;
    figure(3), imshow(uint8(I)), title('before tripple JBF'),drawnow;
    I(:,:,1) = mat2gray(denseMap2)*255;
    I(:,:,2) = mat2gray(addMap)*255;
    figure(4), imshow(uint8(I)), title('after tripple JBF'),drawnow;
    figure(5), imshow(addMap,[]), title('added point'), drawnow;
end