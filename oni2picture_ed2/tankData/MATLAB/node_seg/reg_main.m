 function reg_main
    frameNum = 2;%197-198
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
%% read depth data and tranform to pointcloud
    pc = cell(1,frameNum); D = cell(1,frameNum);
    for i = 1:frameNum
        D{i} = imread(['E:\dataSet\Wajueji_2\processedData\extractdata_afterDRev\d_',int2str(i+196),'.png']);  
        pc{i} = transformUVD2XYZ(double(D{i}), camera_para);
        pc{i} = pcdenoise(pc{i});
    end
    roi = [-230,inf;-inf,inf;0,980];
    pc{1} = select(pc{1},findPointsInROI(pc{1},roi));%place wajueji in ROI
%% hierarchical node
    layers = 4; 
    node_r = [500,250,200,150]/2; %mm. orient to node num = 1,4,8,12
    node_set = cell(1,layers);    %save the node position in every layers 
    for L = 1:layers
        node_set{L} = pcdownsample(pc{1}, 'gridAverage', node_r(L)*2);%grid size should be 2*r
    end
    %%=============figure result of node segmentation=============%%
%     drawNodeSeg(pc,leaf_size,node_set);
%     drawNodeSeg(pc,leaf_size*1.25,node_set);
%% find pointcloud belongs to each node
    pc_set1 = distrPc(pc{1},node_r*1.25,node_set,layers);     %distribute pointcloud
    pc_set2 = distrPc(pc{2},node_r*1.25*1.1,node_set,layers);
    
%% node ICP
    [Tmat, rmse_mat]= hierarchical_ICP(pc_set1,pc_set2,layers);
    
end

function p = transformUVD2XYZ(d, c_pa)
    [H, W] = size(d);
    p_array(:,3) = reshape(d,1,H*W); %z
    for u = 1:W
        for v = 1:H
            p_array((u-1)*H+v,1) = (u - c_pa.cx) * d(v,u)/ c_pa.fx;
            p_array((u-1)*H+v,2) = (v - c_pa.cy) * d(v,u)/ c_pa.fy;
        end
    end
    p_array = p_array(p_array(:,3)>0,:);
    p = pointCloud(p_array);
end

function drawNodeSeg(pc,n_r,node_set)
    [x,y,z] = sphere(15);    
    for L = 1:length(n_r)
        figure(L); pcshow(pc{1});hold on;
        plot3(node_set{L}.Location(:,1),node_set{L}.Location(:,2),node_set{L}.Location(:,3),'r.','markerSize',20);
        for c = 1:node_set{L}.Count
        mesh(x*n_r(L)+node_set{L}.Location(c,1),...
        y*n_r(L)+node_set{L}.Location(c,2),...
        z*n_r(L)+node_set{L}.Location(c,3));
        alpha(0);
        end
        xlabel('X'),ylabel('Y'),zlabel('Z'); title(['layer ',int2str(L)]);
        hold off;
    end
end

function pc_set = distrPc(pc,leaf_size,node_set,layers)
    pc_set = cell(1,layers);
    for L = 1:layers
        pc_set{L} = cell(1,node_set{L}.Count);
        for n = 1:node_set{L}.Count
            [indice, ~] = findNeighborsInRadius(pc,node_set{L}.Location(n,:),leaf_size(L));
            pc_set{L}{n} = select(pc,indice);% pointcloud with radius
            %%===============visualize result===================%%
            p_r = select(pc,indice);
            p_r.Color = repmat(uint8([255,0,0]),p_r.Count,1);
            figure(10),pcshow(pc);hold on;pcshow(p_r);title(['layer=',int2str(L),',the ',int2str(n),'th node']);hold off;
        end
    end
end

function [Tmat, rmse] = hierarchical_ICP(moving_pc,fixed_pc,layers)
    Tmat = cell(1,layers); rmse = cell(1,layers);
    for L = 1:layers
        Tmat{L} = cell(1,size(moving_pc{L},2)); rmse{L} = cell(1,size(moving_pc{L},2));
        for n = 1:size(moving_pc{L},2)
            [Tmat{L}{n},~,rmse{L}{n}] = pcregrigid(moving_pc{L}{n},fixed_pc{L}{n},'Metric','pointToPlane','Verbose',true);
            disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
        end
    end
end

