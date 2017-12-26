%% MAIN
% process whole pipeline of paraFusion, without nodeGraph module and DOF analysis module
%
% fusioned_pc_i:pointcloud fusioned by depthMap from frame_start to the i_th frame
%
% Mention:It is necessary to change your folder into ./node_seg before execute this main.m
% 此main函数，没有nodeGraph管理以及自由度分析模块，只能处理不含自旋转的数据集
function main
    Addpath; close all;  
    global debug_mode; debug_mode = 0;    
    frame_start = 150; frame_end = 199;
    cnt = 1;
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
    para_set = struct('camera_para',camera_para,...
                        'nodeGraph_layers',4,...
                        'node_radius',[500,250,200,150]/2);
    nodeGraph_name = ['nodeGraph_',int2str(frame_start),'_',int2str(frame_end)];
    
    load('./output/result/nodeGraph_150_199.mat');
    tic;
    for i = frame_start:frame_end-1
        %======process canonical frame and the second frame(1-2)======
        if cnt == 1
            D1 = imread(['./input/Wajueji_2/2.0/d_',int2str(i),'.png']);
            D2 = imread(['./input/Wajueji_2/2.0/d_',int2str(i+1),'.png']);
            pc1 = transformUVD2XYZ(D1, camera_para);
            pc2 = transformUVD2XYZ(D2, camera_para);
            [warped_pc, nodeGraph]= process_first_frame(pc1, pc2, para_set, i, cnt);
%             
            eval([nodeGraph_name, '= nodeGraph;']);
            save(['./output/result/',nodeGraph_name,'.mat'],'nodeGraph');

        else %cnt > 1
            %======process two neighboring frame(2-3,3-4,...)======
%             D2 = imread(['./input/Wajueji_2/extractdata_afterDRev/d_',int2str(i+1),'.png']);
            D2 = imread(['./input/Wajueji_2/2.0/d_',int2str(i+1),'.png']);
            pc1 = pcread(['/home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pcd_InfiniTAM/',...
                'fusioned_pc_',int2str(i),'.pcd']);
            pc2 = transformUVD2XYZ(D2, camera_para);
%             load('../mat_data/nodeGraph.mat','nodeGraph_197'); nodeGraph = nodeGraph_197;
            [warped_pc, nodeGraph]= process(pc1, pc2, para_set, nodeGraph, i, cnt);
            
            eval([nodeGraph_name, '= nodeGraph;']);
            save(['./output/result/',nodeGraph_name,'.mat'],'nodeGraph','-append');

        end
        pcwrite(warped_pc,...
                    ['./output/pcd_fromMatlab/warped_pc_',int2str(i),'.pcd'],...
                    'Encoding','ascii');
        disp(['get warped cloud combining from frame',int2str(frame_start),'to frame',int2str(i+1)]);
        
        %======Modified InfiniTAM======
        modified_InfiniTAM(i); % get integrated live pointcloud
        
        %======redistribute node to each points in integrated pointcloud======
        disp(['The ',int2str(cnt+1),'th frame have been integrated into volume!']);
        cnt = cnt + 1; 
        
    end
    toc;
end


% Mention:if cnt = 1, we input warped cloud of frame_1 and the 'cnt_th + 1' depth map
function modified_InfiniTAM(i)
    InfiniTAM_address = '/home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/build/InfiniTAM';

%     arg1 = ' ../Files/wajueji/calib.txt';          %no longer be modified
%     arg2 = ' ../Files/wajueji/Frames_test/%i.ppm'; %no longer be modified
%     arg3 = ' ../Files/wajueji/Frames_test/%i.pgm'; %no longer be modified
    arg1 = ' /home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/Files/wajueji/calib.txt';
    arg2 = [' /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/imageSource/2.0/',...
        int2str(i+1),'.ppm'];                        %no longer be modified
    arg3 = [' /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/imageSource/2.0/',...
        int2str(i+1),'.pgm'];                        %no longer be modified
    arg4 = ' imu_file_not_exist';                    %no longer be modified
    arg5 = [' -pcd_file /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pcd_fromMatlab/warped_pc_',...
        int2str(i),'.pcd'];                          %input address, i.e. warped_pc, of InfiniTAM
    arg6 = [' -output_file_name /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pcd_InfiniTAM/fusioned_pc_',...
        int2str(i+1),'.pcd'];                        %output address of InfiniTAM
    
    dos(['sudo ',InfiniTAM_address, arg1, arg2, arg3, arg4, arg5, arg6]);
end


% function pc = get_warped_pc_file(i)
%     pc = pcread();
% end


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





