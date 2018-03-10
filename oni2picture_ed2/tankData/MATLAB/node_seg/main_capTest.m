%% MAIN_WITH_NODE_EXTEND
% process whole pipeline of paraFusion, containing nodeGraph module and DOF analysis module
%
% fusioned_pc_i:pointcloud fusioned by depthMap from frame_start to the i_th frame
%
% Mention:It is necessary to change your folder into ./node_seg before execute this main.m
% 此main函数，包含nodeGraph管理以及自由度分析模块，可以处理不含自旋转的数据集
function main_capTest
    Addpath; close all; clear all;
    global debug_mode; debug_mode = 0;   
    global dateTime; 
    global figFile; global pcd_fromMatlab_File; global pcd_InfiniTAM;
%     frame_start = 122; frame_end = 199;
    frame_start = 1; frame_end = 50;
    cnt = 49;
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
    para_set = struct('camera_para',camera_para,...
                        'nodeGraph_layers',4,...
                        'node_radius',[1000,250,200,150]/2,...   %[1000,250,200,150]/2
                        'OOR_thres', 500,...
                        'windowSize',3,...
                        'radius_coff',1.30);
    
%     t = fix(clock);
%     dateTime = sprintf('%d%02d%02d_%02d%02d',t(1),t(2),t(3),t(4),t(5));
    dateTime = '20180310_1153';
    load(sprintf('./output/result/nodeGraph_1_50_%s.mat',dateTime));
    
    
    nodeGraph_name = ['nodeGraph_',int2str(frame_start),'_',int2str(frame_end),'_',dateTime];
%     nodeGraph_name = ['nodeGraph_',int2str(1),'_',int2str(50),'_',dateTime];
    DoF_node_relation_map = [];
    
    pcd_fromMatlab_File = sprintf('./output/pcd_fromMatlab/%s',dateTime);
    pcd_InfiniTAM = sprintf('./output/pcd_InfiniTAM/%s',dateTime);
    figFile = sprintf('./output/result/figFile_%s',dateTime);
%     if(~exist(pcd_fromMatlab_File)) mkdir(pcd_fromMatlab_File); end
%     if(~exist(pcd_InfiniTAM)) mkdir(pcd_InfiniTAM); end
%     if(~exist(figFile)) mkdir(figFile); end
    
     
    tic;
    for i = 49%frame_start:frame_end      % i shoule be equal to cnt at the beginning of script
        %======process canonical frame and the second frame(1-2)======
        if cnt == 1
            D1 = imread(['./input/wajueji/data/png/d_',int2str(i),'.png']);
            D2 = imread(['./input/wajueji/data/png/d_',int2str(i+1),'.png']);
            pc1 = transformUVD2XYZ(D1, camera_para);
            pc2 = transformUVD2XYZ(D2, camera_para);
            [warped_pc, nodeGraph, DoF_node_relation_map]= process_first_frame(pc1, pc2, para_set, i, cnt, DoF_node_relation_map);
%
            eval([nodeGraph_name, '= nodeGraph;']);
            if exist(['./output/result/',nodeGraph_name,'.mat'],'file')  % have exist destination file
                delete(['./output/result/',nodeGraph_name,'.mat']);
            end
            save(['./output/result/',nodeGraph_name,'.mat'],'nodeGraph');

        else %cnt > 1
            %======process two neighboring frame(2-3,3-4,...)======
%             D2 = imread(['./input/Wajueji_2/extractdata_afterDRev/d_',int2str(i+1),'.png']);
            D2 = imread(['./input/wajueji/data/png/d_',int2str(i+1),'.png']);
            pc1 = pcread(sprintf('%s/fusioned_pc_%d.pcd',pcd_InfiniTAM,i));
            
            %---------20180306 repalce testFusion as Modified_Infinitam
%             pc1 = pcdownsample(pc1,'gridAverage',3);
            %---------
            
            pc2 = transformUVD2XYZ(D2, camera_para);
             
            pc1 = pcdenoise(pc1);
            pc2 = pcdenoise(pc2);
            [warped_pc, nodeGraph]= process(pc1, pc2, para_set, nodeGraph, i, cnt);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %放到这里是为了让更多的点能参与上面模块的ICP   
%             --------20180120-------
%             while warped_pc.Count > 50000
%                 warped_pc = pcdownsample(warped_pc,'gridAverage',3);
%             end
%             warped_pc = pcdenoise(warped_pc);

            %-----20180124_1950-----
%             if warped_pc.Count > 50000
%                 step = 2;
%                 idx = 1:step:warped_pc.Count;
%                 warped_pc = select(warped_pc,idx);
%             end
%             warped_pc = pcdenoise(warped_pc,'NumNeighbors',2,'Threshold',0.5);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            eval([nodeGraph_name, '= nodeGraph;']);
            save(['./output/result/',nodeGraph_name,'.mat'],'nodeGraph','-append');

        end

        pcwrite(warped_pc,sprintf('%s/warped_pc_%d.pcd',pcd_fromMatlab_File,i),'Encoding','ascii');
        disp(['get warped cloud combining from frame',int2str(frame_start),'to frame',int2str(i+1)]);
        
        %======Modified InfiniTAM======
%         modified_InfiniTAM(i); % get integrated live pointcloud
        test_pc_fusion(i, warped_pc, pc2);
        
        %======redistribute node to each points in integrated pointcloud======
        disp(['The ',int2str(cnt+1),'th frame have been integrated into volume!']);
        cnt = cnt + 1; 
        
    end
    toc;
end


% Mention:if cnt = 1, we input warped cloud of frame_1 and the 'cnt_th + 1' depth map
function modified_InfiniTAM(i)
    global dateTime;
    InfiniTAM_address = '/home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/build/InfiniTAM';

%     arg1 = ' ../Files/wajueji/calib.txt';          %no longer be modified
%     arg2 = ' ../Files/wajueji/Frames_test/%i.ppm'; %no longer be modified
%     arg3 = ' ../Files/wajueji/Frames_test/%i.pgm'; %no longer be modified
    arg1 = ' /home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/Files/wajueji/calib.txt';
    arg2 = [' /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/input/wajueji/data/ppmpgm/c_',...
        int2str(i+1),'.ppm'];                        %no longer be modified
    arg3 = [' /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/input/wajueji/data/ppmpgm/d_',...
        int2str(i+1),'.pgm'];                        %no longer be modified
    arg4 = ' imu_file_not_exist';                    %no longer be modified
    arg5 = [' -pcd_file /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pcd_fromMatlab/',dateTime,'/warped_pc_',...
        int2str(i),'.pcd'];                          %input address, i.e. warped_pc, of InfiniTAM
    arg6 = [' -output_file_name /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pcd_InfiniTAM/',dateTime,'/fusioned_pc_',...
        int2str(i+1),'.pcd'];                        %output address of InfiniTAM
    arg7 = sprintf(' -volume_size 3');
    arg8 = sprintf(' -volume_resolution 512');
    dos(['sudo ',InfiniTAM_address, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8]);
end


function p = transformUVD2XYZ(d, c_pa)
    [H, W] = size(d);
    d = double(d);
    p_array(:,3) = reshape(d,H*W,1);        %z can obtain from var d 
    for u = 1:W
        for v = 1:H
            p_array((u-1)*H+v,1) = (u - c_pa.cx) * d(v,u)/ c_pa.fx;
            p_array((u-1)*H+v,2) = (v - c_pa.cy) * d(v,u)/ c_pa.fy;
        end
    end
    index = p_array(:,3)>0;
 
    p_array = p_array(index,:); 
    p = pointCloud(p_array);
end


function test_pc_fusion(i, pc1, pc2)
    global pcd_InfiniTAM;
    if(mod(i,5)==0) pc1 = pcdownsample(pc1,'gridAverage',1.5); end
    
    pc2 = pcdownsample(pc2,'gridAverage',3);
    array = [pc1.Location; pc2.Location];
    pc_fusioned = pointCloud(array);
%     pc_fusioned = pcdownsample(pc_fusioned, 'gridAverage', 3);
%     pc_fusioned = pcdenoise(pc_fusioned);
    pcwrite(pc_fusioned, sprintf('%s/fusioned_pc_%s.pcd',pcd_InfiniTAM,int2str(i+1)));
end


