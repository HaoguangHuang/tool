%% MAIN
% process whole pipeline of paraFusion
%
% Mention:It is necessary to change your folder into ./node_seg before execute this main.m
function main
    Addpath; close all;  
    global debug_mode; debug_mode = 0;    
    frame_start = 197; frame_end = 200;
    cnt = 2;
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
    
    for i = frame_start:frame_end
        %======process canonical frame and the second frame(1-2)======
        if cnt == 1
            D1 = imread(['./input/Wajueji_2/extractdata_afterDRev/d_',int2str(i),'.png']);
            D2 = imread(['./input/Wajueji_2/extractdata_afterDRev/d_',int2str(i+1),'.png']);
            pc1 = transformUVD2XYZ(D1, camera_para);
            pc2 = transformUVD2XYZ(D2, camera_para);
            warped_pc = process_first_frame(pc1, pc2, camera_para);
            pcwrite(warped_pc,...
                    ['./output/pcd_fromMatlab/pc_','1','.pcd'],...
                    'Encoding','ascii');
        else %cnt > 1
            %======process two neighboring frame(2-3,3-4,...)======
            D2 = imread(['./input/Wajueji_2/extractdata_afterDRev/d_',int2str(i+1),'.png']);
            pc1 = get_warped_pc_file(cnt);
            pc2 = transformUVD2XYZ(D2, camera_para);
            warped_pc = process(pc1, pc2, camera_para);
            pcwrite(warped_pc,...
                    ['./output/pcd_fromMatlab/pc_',int2str(cnt),'.pcd'],...
                    'Encoding','ascii');
        end
        
        modified_InfiniTAM(cnt); % get integrated live pointcloud
        
        %======redistribute node to each points in integrated pointcloud======
        
        
        cnt = cnt + 1; 
    end
end


% Mention:if cnt = 1, we input warped cloud of frame_1 and the 'cnt_th + 1' depth map
function modified_InfiniTAM(cnt)
    InfiniTAM_address = '/home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/build/InfiniTAM';

    arg1 = ' ../Files/wajueji/calib.txt';          %no longer be modified
%     arg2 = ' ../Files/wajueji/Frames_test/%i.ppm'; %no longer be modified
%     arg3 = ' ../Files/wajueji/Frames_test/%i.pgm'; %no longer be modified
    arg2 = ' /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/imageSource/test_197_200/%i.ppm'; %no longer be modified
    arg3 = ' /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/imageSource/test_197_200/%i.pgm'; %no longer be modified
    arg4 = ' imu_file_not_exist';                  %no longer be modified
    arg5 = [' -pcl_file /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pcd_fromMatlab/pc_',...
        int2str(cnt),'.pcd'];
    
    dos([InfiniTAM_address, arg1, arg2, arg3, arg4, arg5]);
end


function pc = get_warped_pc_file(cnt)
    pc = pcread();
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





