%% SPARSE2DENSE
% �Ե�һ֡ͼΪguidance����ͼ���в�ֵ��ͼһ��û��unique_corr�ĵ��correspondence
%         pc1:pointCloud.      guidance���ƣ���Ҫ��ת��pc2�����꣬Ȼ���3DͶӰ��2D���ƽ��
%         pc2:pointCloud.      �������ĵ��ƣ���Ҫ��3DͶӰ��2D���ƽ��
% point_corr:pc1.Count*3.     [indice, correspondence, dist]
% camera_para:struct.          containing fx, fy, cx, cy
%    densePC2:640*480 array.   ��ֵ����ܵ�pointCloud1�����ͼ  
%        Tmat:cell.            save ICP result of each node in each layers
% pc_bestNode_distr: array.    n*1.  ��¼ÿ��point���������node���ĸ�
%
% Note:˼·����ͼһΪguidance������ͼ������������guided JBF���������£�
%      1.pc1����unique_corr�Լ�û��unique_corr�ĵ㣬�ֱ�transform��pc2�����ϣ��õ�mask1��mask0.
%      2.mask = (~mask1) & mask0. ��������mask==1�ĵ㣬�˴�������z-buffer
%      3.tripple_filter�У�ֻ�Է��δ�����depth~=0����ȵ���м�Ȩƽ�����˴�Ҫ������mask==1�����ص㶼Ҫ������ֵ
%      tripple_filter
%      a) ��һ֡ͼ�е�ĳ�� i (�õ㲻ӵ��unique_correspondence)��Ϊ���ģ� ��widthΪ�߳����췽�δ���
%      b) ���������ص��ȨֵΪ: w = w_intensity * w_distance * w_depth
%      c) �� i �����ֵΪ d_i = ����_j * d_j / ����_j

function denseMap2 = sparse2dense(pc1, pc2, point_corr, camera_para, Tmat, pc_bestNode_distr)
    if nargin < 1, load('pc.mat','pc_197'); pc1 = pc_197; clear pc_197; end;
    if nargin < 2, load('pc.mat','pc_198'); pc2 = pc_198; clear pc_198; end;
    if nargin < 3, load('corrIndex.mat','corrIndex_wc03_thres2');
        point_corr = corrIndex_wc03_thres2;  clear corrIndex_wc03_thres2; 
    end
    if nargin < 4, camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202); end
    if nargin < 5, load('Tmat_wc03.mat'); end
    if nargin < 6, load('pc_bestNode_distr.mat');
        pc_bestNode_distr = pc_bestNode_distr_wc03; clear pc_bestNode_distr_wc03; 
    end
    %%======��pc1��unique_corrͶӰ��pc2=======
    pc1_inCoo2 = transform_pc1_into_coordinate_of_pc2(pc1, Tmat, pc_bestNode_distr);
    
    %%======ͶӰ����POI�˲���pc1_inCoo2�����ƽ��======
    unique_corr = point_corr(point_corr(:,2)>0,:);
    not_unique_corr = point_corr(point_corr(:,2)==0,:);
    
    xyz_pc0 = select(pc1_inCoo2, not_unique_corr(:,1));  %֮ǰƥ���corr��ľ��볬��2mm
    y_pc0 = xyz_pc0.Color(:,1); xyz_pc0 = xyz_pc0.Location(:,:);
    %D0:pc1_inCoo2������û��unique_correspondence�ĵ��ͶӰuvd
    %Y0:pc1_inCoo2������û��unique_correspondence�ĵ��ͶӰuvY
    [~, D0, Y0]= transformXYZ2UVDI(xyz_pc0, camera_para, y_pc0);
    mask0 = D0 > 0;
    
    xyz_pc1 = select(pc1_inCoo2, unique_corr(:,1));  %20342  
    y_pc1 = xyz_pc1.Color(:,1); xyz_pc1 = xyz_pc1.Location(:,:); 
    
    %D1:pc1_inCoo2������ӵ��unique_correspondence�ĵ��ͶӰuvd
    %Y1:pc1_inCoo2������ӵ��unique_correspondence�ĵ��ͶӰuvY
    [~, D1, Y1]= transformXYZ2UVDI(xyz_pc1, camera_para, y_pc1);
    mask1 = D1 > 0;  %20280
    
    %mask>0�ĵ���Ҫtripple_filter����ĵ�
    mask = (~mask1) & mask0;
    
    %D2:pointCloud2������ӵ��unique_correspondence�ĵ��ͶӰuvd
    %Y2:pointCloud2������ӵ��unique_correspondence�ĵ��ͶӰuvY
    xyz_pc2 = select(pc2,unique_corr(:,2));
    y_pc2 = xyz_pc2.Color(:,1); xyz_pc2 = xyz_pc2.Location(:,:);
    [~, D2, Y2]= transformXYZ2UVDI(xyz_pc2, camera_para, y_pc2);
    
    [denseMap2, addMap] = tripple_guided_JBF(D1, D2, Y1, Y2, mask);  %denseMap�ǳ��ܵ�D2
   
end


function [vud_pc, D, Y]= transformXYZ2UVDI(xyz_pc, camera_para, y_pc1)
    D = zeros(480,640);  Y = zeros(480,640);
    vud_pc = zeros(size(xyz_pc));
    vud_pc(:,3) = xyz_pc(:,3);                                                       %d = z
    count = 0;
    for i = 1:size(xyz_pc,1)
        if vud_pc(i,3) == 0, continue; end
        vud_pc(i,2) = round(xyz_pc(i,1)*camera_para.fx/xyz_pc(i,3)+camera_para.cx);  %u = (x*f_x)/z + c_x
        vud_pc(i,1) = round(xyz_pc(i,2)*camera_para.fy/xyz_pc(i,3)+camera_para.cy);  %v = (y*f_y)/z + c_y
        D(vud_pc(i,1),vud_pc(i,2)) = vud_pc(i,3);
        Y(vud_pc(i,1),vud_pc(i,2)) = y_pc1(i,1);
        count = count + 1;
    end
end

%% TRANSFORM_PC1_INTO_COORDINATE_OF_PC2
function pc1_inCoo2 = transform_pc1_into_coordinate_of_pc2(pc1, Tmat, pc_bestNode_distr)
    xyz_pc1_inCoo2 = zeros(pc1.Count,3);
    for i = 1:pc1.Count
        if pc_bestNode_distr(i,1) == 0, continue; end;  %exclude the poins that belonging to no node 
        p = pc1.Location(i,:);
        T = Tmat{4}{pc_bestNode_distr(i,1)};
        p_afterTran = transformPointsForward(T, p);
        xyz_pc1_inCoo2(i,:) = p_afterTran;
    end
    pc1_inCoo2 = pointCloud(xyz_pc1_inCoo2);
    pc1_inCoo2.Color = pc1.Color;
end







