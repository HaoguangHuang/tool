%compare pc197, pc200, and fusioned_pc_200

function compare_fusioned_non_fusion
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
    
    for frame_no = 12:200
        d = imread(sprintf('./input/Wajueji_2/dRecvy_use_new_guide2/d_%d.png',frame_no));
    %     d = imread(sprintf('./input/Wajueji_2/2.0/d_%d.png',frame_no));
        d_yuantu = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/extractdata_afterDRev/d_%d.png',frame_no));
        pc = transformUVD2XYZ(d,camera_para); %G
    %     pc_extracted = transformUVD2XYZ(d_yuantu,camera_para);
        figure(2),pcshow(pc),title(sprintf('depthRecvy depth %d',frame_no));
    %     figure(3),pcshow(pc_extracted),title(sprintf('pc\_extracted depth %d',frame_no));

        mask = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/1_200/best_mask/mask_%d.png',...
            frame_no));
        fg_d = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/depth/fusionedForegroundData/fusionedForegroundData%d.png',...
                frame_no));
        d_yuantu_mask = fg_d .* uint16(mask>0);
        pc_yuantu_mask = transformUVD2XYZ(d_yuantu_mask,camera_para);
        figure(4),pcshow(pc_yuantu_mask),title('pc\_yuantu\_mask');
    end
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