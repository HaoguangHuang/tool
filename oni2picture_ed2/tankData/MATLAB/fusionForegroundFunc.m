% 以fusionForegroundSeg帧为单位，单位内的帧进行融合
function fusionedForegroundData = fusionForegroundFunc(foregroundData, fusionForegroundSeg)
    frameNum = size(foregroundData,2);
    if mod(frameNum, fusionForegroundSeg) ~= 0
        disp('fusionForegroundFunc error: frameNum不整除fusionForegroundSeg');
        return;
    end
    unitNum = frameNum / fusionForegroundSeg;
    result = {};
    matlabpool 4;
%     delete(gcp);
%     parpool('local', 4);
    parfor j = 1:unitNum
        for u = 1:640
            for v = 1:480
                tmp = double(0); count = 0;
                for i = (fusionForegroundSeg*(j - 1)+1):(fusionForegroundSeg*j)
                    if foregroundData(i).data(v,u)~=0
                        tmp = tmp + double(foregroundData(i).data(v,u));
                        count = count + 1;
                    end
                end
                if count == 0   %像素点深度值恒为0
                    result(j).data(v,u) = 0; continue;
                else
                    result(j).data(v,u) = double(tmp)/count;
                end
            end
        end
    end
     matlabpool close;
%     delete(gcp);
    fusionedForegroundData = result;
end