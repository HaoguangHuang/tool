function mask_gbf = guidedBilateralFilter(mask, fusionedForegroundData)
    [row col]  =size(mask);
    windows_width = 11;%奇数容易分配num
    windows_height = 11;
    sigma = 7;
    window = ones(windows_height,windows_width);
    sigma_precompute = -2*sigma*sigma;
    r_start = round((windows_height + 1)/2);
    r_end = round(row - (windows_height+1)/2);
    c_start = round((windows_width + 1)/2);
    c_end = round(col - (windows_width + 1)/2);
    num =(r_end - r_start + 1)*(c_end - c_start + 1);%待处理的pixel数目
    %预分配空间
    G1_array = zeros(size(window));%存放 (i - j)^2
    G2_array(num).data = zeros(size(window));%存放 (y_i - y_j)^2
    
    %%计算G1_array
    G1_array = computeDistance(windows_width, windows_height);
    
    for r = r_start : r_end
        for c = c_start : c_end
            G1_array()
        end
    end
end


function g1 = computeDistance(windows_width, windows_height)
    g1 = zeros(windows_height, windows_width);
    origin = [(windows_height+1)/2, (windows_width+1)/2];
    for r = windows_height
        for c = windows_width
            g1(r,c) = 
        end
    end
end