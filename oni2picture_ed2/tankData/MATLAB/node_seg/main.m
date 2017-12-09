%% MAIN
% process whole pipeline of paraFusion

function main
    Addpath; close all;  
    global debug_mode; debug_mode = 1;    
    frame_start = 197; frame_end = 200;
    cnt = 1;
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
    
    for i = frame_start:frame_end
        %======process canonical frame and the second frame(1-2)======
        if cnt == 1
            pc1 = transformUVD2XYZInten();
            cnt = cnt + 1;
        end
    
        %======process two neighboring frame(2-3,3-4)======
        %cnt > 1
        
        cnt = cnt + 1;
    end
end


function modified_InfiniTAM(~)
    InfiniTAM_address = '/home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/build/InfiniTAM';

    arg1 = ' ../Files/wajueji/calib.txt';          %no longer be modified
    arg2 = ' ../Files/wajueji/Frames_test/%i.ppm'; %no longer be modified
    arg3 = ' ../Files/wajueji/Frames_test/%i.pgm'; %no longer be modified
    arg4 = ' imu_file_not_exist';                  %no longer be modified
    arg5 = ' -pcl_file /home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pointcloud/pc_1.pcd';
    
    dos([InfiniTAM_address, arg1, arg2, arg3, arg4, arg5]);
end

