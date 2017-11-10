%% CHECK_TRANSFORMATION_MAP
% check transformation map is true or not
% 
% Note:attemp to tranform all the points in projected pc2 into coo of pc1

function check_transformation_map()
    load('./mat_data/test.mat','transformationMap');
%     transformationMap = transformationMap2;
    load pc.mat; 
%     pc1 = pc_197; clear pc_197;
    pc2 = pc_198; clear pc_198;
    global camera_para;
    camera_para= struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
    %======project pc2======
    pc2_xyz = pc2.Location(:,:);
    D2 = transformXYZ2UVD(pc2_xyz,camera_para);
    
%     count = 0;
%     for r = 1:480
%         for c = 1:640
%             if sum(transformationMap(r,c,:))~=0, count = count + 1; end
%         end
%     end
%     display(sum(sum(transformationMap(:,:,1)~=0)));
%     display(count);
    %======use D2 and transformationMap to get interpolated pc2 in coo of pc1======
    use_transformationMap(D2,transformationMap);
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

function use_transformationMap(D,transformationMap)
    D1_from_D2 = zeros(480*640,3);
    for r = 1:480
        for c = 1:640
            a = transformationMap(r,c,1); b = transformationMap(r,c,2); g = transformationMap(r,c,3);
            t1 = transformationMap(r,c,4); t2 = transformationMap(r,c,5); t3 = transformationMap(r,c,6);   
            T1 = [a, b, g, t1, t2, t3];
            d = D(r,c);
            if d~=0 && sum(T1)~=0
                X2 = transformUVD2XYZ(r,c,d);
                R = rodrigues([a,b,g]);
                t = [t1; t2; t3];
                X1 = inv(R)*(X2-t);  %Mention : here should be inv(R)*(x+t), because of the format of affine3d
                D1_from_D2((r-1)*480+c,:) = X1';
            end
        end
    end
    pc1_from_D2 = pointCloud(D1_from_D2);
    figure(12),pcshow(pc1_from_D2);title('after transformationMap, pc2 in coo of pc1');
end

function X = transformUVD2XYZ(v,u,d)
    global camera_para;
    z = d;
    x = (u - camera_para.cx)*z/camera_para.fx;
    y = (v - camera_para.cy)*z/camera_para.fy;
    X = [x; y; z];
end

