%% FINDCORR_WITHINTEN:find correspondence with geometry information and color intensity information
%              ptCloudB:pointCloud. pc2. fixing point
%              ptCloudA:pointCloud. pc1. moving point
%                  locA:array.      data from pc1.Location, after initial transform
%                 dists:scalar.     containing geometry and color intensity information
%               indices:scalar.     record the correspondence
% Note: Here ptCloudA only provide color intensity information. xyz in ptCloudA haven't update according to transformation from last 
%       iteration.
function [indices, dists] = findCorr_withInten(ptCloudB, locA, ptCloudA)
    if nargin < 4, w_c = 0.3; end %weight of color intensity
    w_d = 1 - w_c;                %weigh of depth
    %%=====brute force======
    xyzi_1 = [locA,double(ptCloudA.Color(:,1))]; xyzi_2 = [ptCloudB.Location, double(ptCloudB.Color(:,1))];
    num1 = size(xyzi_1,1);               num2 = size(xyzi_2,1);
    indices = zeros(num1, 1); dists = zeros(num1, 1);
    tic;
    for i = 1:num1
        E_d = zeros(num2,1); 
        for j = 1:3               %xyz, 3 dimension
            E_d = E_d + (xyzi_1(i,j)-xyzi_2(:,j)).^2;
        end
        E_d = w_d * mapminmax(E_d,0,1);
        E_c = (xyzi_1(i,4)-xyzi_2(:,4)).^2;
        E_c = w_c * mapminmax(E_c,0,1);
        E = E_d + E_c;            %E = w_d*sqrt((x1-x2)^2 + (y1-y2)^2+(z1-z2)^2) + w_c*abs(I1-I2)
        [dists(i), indices(i)] = min(E);
    end
    toc;
end