%% FIND_UNIQUE_CORR:find the unique correspondence between points in pc1 and those in pc2
%   pc_bestNode_distr:array.       Its size is pc1.Count*1. Record points belong to which best node
%                tmat:cell.        record ICP result in the last layers. Element is a cell containing an affine3d object
%                 pc1:pointCloud.  moving pointcloud
%                 pc2:pointCloud.  fixed pointcloud
%              layers:scalar.      number of node layer
%           corrIndex:array.       Its size is pc1.Count*3. [indice, correspondence, dist]
function corrIndex = find_unique_corr(pc_bestNode_distr, pc1, pc2, tmat, thres_corr)
    if nargin < 1, load('pc_bestNode_distr.mat','pc_bestNode_distr_wc03');pc_bestNode_distr = pc_bestNode_distr_wc03;end
    if nargin < 2, load('pc.mat','pc_197'); pc1 = pc_197; end
    if nargin < 3, load('pc.mat','pc_198'); pc2 = pc_198; end
    if nargin < 4, load('Tmat.mat','Tmat_wc03'); tmat = Tmat_wc03{end}; end
    if nargin < 5, thres_corr = 2;end       %mm
    p_node = linspace(1,size(pc_bestNode_distr,1),size(pc_bestNode_distr,1))'; p_node = [p_node, pc_bestNode_distr];
    not_zero_index = p_node(:,2)>0;
    p_node = p_node(not_zero_index,:);      %now p_node doesn't have point belonging to none node
    hash = cell(1,pc2.Count);               %hash store the nearset point of p2 in pc1
                                            %eg:hash{3} = [100,35]
                                            %   which means the closest point from pc1 to the 3th point in pc2 now
                                            %   is the 100th point, and their distance is 35mm
                                     
    for i = p_node(:,1)'
        p1 = pc1.Location(i,:); T1 = tmat{pc_bestNode_distr(i)};
        p1_inPc2 = transformPointsForward(T1, p1);%transform p1 from pc1 coo to pc2 coo
        [indice, dist] = findNearestNeighbors(pc2, p1_inPc2, 1);
        if dist >= thres_corr, continue; end
        if isempty(hash{indice}), hash{indice} = [i, dist]; continue; 
        else
           e = hash{indice};
           if dist < e, hash{indice} = [i,dist]; continue; end
        end
    end

    
    %%======transform corrIndex format to 
    corrIndex = zeros(pc1.Count,3);
    corrIndex(:,1) = linspace(1,pc1.Count,pc1.Count)';
    for i = 1:size(hash,2)
        if isempty(hash{i}), continue; 
        else
            a = hash{i};
            corrIndex(a(1),:) = [a(1),i,a(2)];
        end
    end
end