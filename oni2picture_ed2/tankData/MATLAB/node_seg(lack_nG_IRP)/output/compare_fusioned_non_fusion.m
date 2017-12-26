%compare pc197, pc200, and fusioned_pc_200

function compare_fusioned_non_fusion
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
    
%     d_199 = imread('./input/Wajueji_2/extractdata_afterDRev/d_197.png');
    d_199 = imread('./input/Wajueji_2/2.0/d_199.png');
%     pc_197 = transformUVD2XYZ(d_197,camera_para); %R
    pc_199 = transformUVD2XYZ(d_199,camera_para); %G
    
    pc_fusioned_150_199 = pcread('./output/pcd_InfiniTAM/150_199/fusioned_pc_199.pcd');%B
    
%     pc_197.Color = repmat(uint8([255,0,0]),pc_197.Count,1);
    pc_199.Color = repmat(uint8([0,255,0]),pc_199.Count,1);
    pc_fusioned_150_199.Color = repmat(uint8([0,0,255]),pc_fusioned_150_199.Count,1);
    
%     figure(1),pcshow(pc_197),hold on, pcshow(pc_199),hold off, title('pc198(G) and pc_fusioned_150_199(B)');
    figure(2),pcshow(pc_199),hold on, pcshow(pc_fusioned_150_199),hold off, title('pc199(G) and pc\_fusioned\_150\_199(B)');
end



% camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);

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