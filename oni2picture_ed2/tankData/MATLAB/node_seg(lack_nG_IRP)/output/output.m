%% OUTPUT
%  For fusion: This function output two depth map and six 6FoD transformation map, containing Euler angle and translation.
%  depthMap : .png format. contain all points from pointcloud1 and pointcloud2.
%  transformationMap : .txt. contain 4*3 [R;t] of the non_empty pixels of the second image. 
%                      eg. You can find the transformation of (r,c) in the second image by search (r-1)*480+c
function output(output_filename, D1, D2, camera_para, transformationMap)
    if nargin < 1, output_filename = './node_seg/output/res'; end
    if nargin < 2, load('./mat_data/pc.mat','pc_197');  pc1 = pc_197; clear pc_197; end
    if nargin < 3, load('./mat_data/pc.mat','pc_198');  pc2 = pc_198; clear pc_197; end
    if nargin < 4, camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202); end
    if nargin < 5, load('./mat_data/transformationMap.mat','transformationMap_197_198'); 
        transformationMap = transformationMap_197_198;
    end
    H  =480; W = 640;
    
    %======if not get D1, D2 as input, project pc1, pc2 from 3D to 2D======
    D1 = transformXYZ2UVD(pc1.Location(:,:), camera_para);
    D2 = transformXYZ2UVD(pc2.Location(:,:), camera_para);
    
    %======construct a [H*W*4,3] matrix to represent transformationMap======
    T_data = zeros(H*W*4,3);
    mask = D2 > 0;
    for r = 1:H
        for c = 1:W
            if mask(r,c) == 1   
                T = abgt123_to_T(transformationMap(r,c,:));
            else  %mask(r,c) == 0
                T = zeros(4,3);
            end
            num = (r-1)*H + c;
            start = 4*num - 3;
            T_data(start:start+3,1:3) = T;
        end
    end
    
    %======output D1,D2,T_data to specified file======
    imwrite(uint16(D1),[output_filename,'/197.png']);
    imwrite(uint16(D2),[output_filename,'/198.png']);
    
    file_in = fopen([output_filename,'/transformationMap_197_198.txt'],'wt');
    [row,~] = size(T_data);
    for i = 1:row
        data = T_data(i,:);
        fprintf(file_in,'%6.4f %6.4f %6.4f\n',data);
    end
    fclose(file_in);
end



% Mention : Here T is size of 4*3
function T = abgt123_to_T(abgt123)
    a = abgt123(1,1,1); b = abgt123(1,1,2); g = abgt123(1,1,3);
    R = rodrigues([a,b,g]);
    t = [abgt123(1,1,4),abgt123(1,1,5),abgt123(1,1,6)];
    T = [R;t];
end

function D= transformXYZ2UVD(xyz_pc, camera_para)
    D = zeros(480,640);  
    vud_pc = zeros(size(xyz_pc));
    vud_pc(:,3) = xyz_pc(:,3);                                                       %d = z
    for i = 1:size(xyz_pc,1)
        if vud_pc(i,3) == 0, continue; end
        vud_pc(i,2) = round(xyz_pc(i,1)*camera_para.fx/xyz_pc(i,3)+camera_para.cx);  %u = (x*f_x)/z + c_x
        vud_pc(i,1) = round(xyz_pc(i,2)*camera_para.fy/xyz_pc(i,3)+camera_para.cy);  %v = (y*f_y)/z + c_y
        D(vud_pc(i,1),vud_pc(i,2)) = vud_pc(i,3);
    end
end
