%% VISUALIZE_WITH_CORR:visualize pc1 in coordinate of pc2 after transformation, within the unique transformation
%                     pc1:pointCloud.  Moving pointcloud
%                     pc2:pointCloud.  Fixed pointcloud
%               corrIndex:array.       Its size is pc1.Count*3. [p1, corresponedence in pc2, dist]
%                    tmat:cell.        Record ICP result in the last layers. Element is a cell containing an affine3d object
%       pc_bestNode_distr:array.       pc1.Count*1. Record which node have best connection with point in pc1 
% Note: Here we only visualize the point in pc1 having unique correspondence via a series of line segment 
function visualize_with_corr(pc1, pc2, corrIndex, tmat, pc_bestNode_distr)
    if nargin < 1, load('pc.mat','pc_197'); pc1 = pc_197; end
    if nargin < 2, load('pc.mat','pc_198'); pc2 = pc_198; end
    if nargin < 3, load('corrIndex.mat','corrIndex_wc03_thres2'); corrIndex = corrIndex_wc03_thres2;end
    if nargin < 4, load('Tmat.mat','Tmat_wc03'); tmat = Tmat_wc03; tmat = tmat{4};end
    if nargin < 5, load('pc_bestNode_distr.mat','pc_bestNode_distr_wc03'); 
                   pc_bestNode_distr = pc_bestNode_distr_wc03;end
    
    index = corrIndex(:,2)>0;              %exclude point having no correspondence
    pc1_ex0 = corrIndex(index,:); num_ex0 = size(pc1_ex0,1);
    pc1_inCoo2 = zeros(num_ex0,3);
    for i = 1:num_ex0
        point1 = pc1.Location(pc1_ex0(i,1),:); n = pc_bestNode_distr(pc1_ex0(i,1));
        point1_InCoo2 = transformPointsForward(tmat{n},point1);
        pc1_inCoo2(i,:) = point1_InCoo2;   %array
    end
    global debug_mode;
    if debug_mode
    pc1_inCoo2 = pointCloud(pc1_inCoo2);   %pointCloud, only contain point have unique best correspondence
    pc1_inCoo2.Color = repmat(uint8([0,255,0]),pc1_inCoo2.Count,1);
    figure(103),pcshow(pc1_inCoo2),hold on; pcshow(pc2);title('pc1(green),pc2(red), in Coo of pc2');hold off;
    
    pc1_vis = pc1; pc1_vis.Color = repmat(uint8([0,255,0]),pc1_vis.Count,1);
    figure(104),pcshow(pc1_vis),hold on; pcshow(pc2);title('pc1(green),pc2(red), pc1 in Coo of pc1, pc2 in Coo pc2');hold off;
    end
    
    %%============line the correspondence============
    figure(106),pcshow(pc1_inCoo2),hold on; pcshow(pc2);title('pc1(green),pc2(red), in Coo of pc2');
    for i = 1:pc1_inCoo2.Count
        x = [pc1_inCoo2.Location(i,1), pc2.Location(pc1_ex0(i,2),1)];
        y = [pc1_inCoo2.Location(i,2), pc2.Location(pc1_ex0(i,2),2)];
        z = [pc1_inCoo2.Location(i,3), pc2.Location(pc1_ex0(i,2),3)];
        line(x,y,z,'LineStyle','-')
    end
    hold off;
    
end