%% Teapot Test
camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);

pc = pcread('teapot.ply');
array = [pc.Location(:,1),pc.Location(:,3),pc.Location(:,2)]*0.25*1000;
pc = pointCloud(array);

offset = [0,-0.3,1.5]*1000; %from global coo to camera coo

figure(53),pcshow(pc);hold on;
plot3(offset(1),offset(2),offset(3),'r.','MarkerSize',20);
hold off;
xlabel('x'),ylabel('y'),zlabel('z');
title('teapot in global coo');


teapot_g = pc.Location;
teapot_c = teapot_g + repmat(offset,pc.Count,1); %% save 1

figure(54),pcshow(pointCloud(teapot_c));hold on;
plot3(0,0,0,'r.','MarkerSize',20);
hold off;
xlabel('x'),ylabel('y'),zlabel('z');
title('teapot in camera coo');

roi = [-inf,inf;-inf,inf;0,1.6*1000];
teapot_roi = pointCloud(teapot_c);
teapot_roi = select(teapot_roi,findPointsInROI(teapot_roi,roi)); % place wajueji in ROI
figure(56),pcshow(teapot_roi);hold on;
plot3(0,0,0,'r.','MarkerSize',20);
hold off;
xlabel('x'),ylabel('y'),zlabel('z');
title('teapot_ROI in camera coo');

% 
D = zbuffer_forward_proj(teapot_roi.Location,camera_para);
% D_output = imrotate(D,180); %% save 2
D_output = D;
figure(44),imshow(D_output,[]),title('projected teapot');

pc_visible = transformUVD2XYZ(D_output,camera_para);
figure(55),pcshow(pc_visible),title('pc invisible');

imwrite(uint16(D_output),'./input/teapot/tp.pgm');
pcwrite(pointCloud(teapot_c),'./input/teapot/tp.pcd');

function D = zbuffer_forward_proj(pc,camera_para)
    D = ones(480,640)*inf;
    cnt = 0; %count the nonunique_nonproj points
    for i = 1:size(pc,1)
        x = pc(i,1); y = pc(i,2); z = pc(i,3);
        u = round(x/z*camera_para.fx + camera_para.cx + 0.5);
        v = round(y/z*camera_para.fy + camera_para.cy + 0.5);
        d = z;
        if u < 1 || u > 640 || v < 1 || v > 640, continue; end
        if D(v,u) > d % the i_th point is not occluded
            D(v,u) = d;
        end    
        cnt = cnt + 1; 
    end
    D(D==inf) = 0;
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