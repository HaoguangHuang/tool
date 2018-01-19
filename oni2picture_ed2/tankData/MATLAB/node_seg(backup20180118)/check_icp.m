%% CHECK_ICP: Check whether the ICP result of every node is correct or not
%               Tmat: cell.           Tmat{layers}{node_transformation}
%                pc1: pointCloud.     moving point
%                pc2: pointCloud.     fixing point
%  pc_bestNode_distr: array.          pc1.Count*1; 
function check_icp(Tmat, pc1, pc2, pc_bestNode_distr)
    if nargin < 1, load('Tmat_wc03.mat','Tmat'); end;
    if nargin < 2, load('pc.mat','pc_197'); pc1 = pc_197; end;
    if nargin < 3, load('pc.mat','pc_198'); pc2 = pc_198; end;
    if nargin < 4, load('pc_bestNode_distr.mat','pc_bestNode_distr_wc03'); 
        pc_bestNode_distr = pc_bestNode_distr_wc03;
    end
    
    nodeNum = size(Tmat{4},2);
    indice  = zeros(pc1.Count, 2);  indice(:,1) = linspace(1,pc1.Count,pc1.Count);
    for i = 1:nodeNum
        tmat      = Tmat{4}{i};               %affine3d
        indice1    = indice(pc_bestNode_distr == i,1);  %find points in pc1 belonging to the i_th node
        pc_preT   = select(pc1, indice1);     %pointcloud before transformation
        pc_preT.Color = repmat(uint8([0,255,0]),pc_preT.Count,1);
        pc_afterT = pctransform(pc_preT, tmat);
        figure(1);
        subplot(1,2,1), pcshow(pc2), hold on, pcshow(pc_preT), hold off;
        title(['result of the ', int2str(i), 'th node, pc\_preT']);
        subplot(1,2,2), pcshow(pc2), hold on, pcshow(pc_afterT), hold off;
        title(['result of the ', int2str(i), 'th node, pc\_afterT']);
    end
end