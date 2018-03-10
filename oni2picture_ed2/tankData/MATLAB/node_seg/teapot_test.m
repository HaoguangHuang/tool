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

% roi = [-inf,inf;-inf,inf;0,1.6*1000];
% teapot_roi = pointCloud(teapot_c);
% teapot_roi = select(teapot_roi,findPointsInROI(teapot_roi,roi)); % place wajueji in ROI

figure(66),pcshow(pointCloud(teapot_c));hold on;
plot3(0,0,0,'r.','MarkerSize',20);
hold off;
xlabel('x'),ylabel('y'),zlabel('z');
title('teapot_ROI in camera coo');

% 
D = zbuffer_forward_proj(teapot_c,camera_para);
% D_output = imrotate(D,180); %% save 2
D1 = D;
figure(44),imshow(D1,[]),title('projected teapot,zbuffer');
pc_visible = transformUVD2XYZ(D1,camera_para);
figure(55),pcshow(pc_visible),title('pc visible,zbuffer');

step = 15;
D2 = zbuffer_forward_proj_pointSpliting(teapot_c,camera_para,step);
figure(45),imshow(D2,[]),title('projected teapot,point spliting');
pc_pointSpling = transformUVD2XYZ(D2,camera_para);
figure(56),pcshow(pc_pointSpling),title(sprintf('pc pointsplting,step=%d',step));


% imwrite(uint16(D_output),'./input/teapot/tp.pgm');
% pcwrite(pointCloud(teapot_c),'./input/teapot/tp.pcd');

D(D>0) = D(D>0) - 200;
imwrite(uint16(D),'./input/teapot/tp_plus10.pgm');

function D = zbuffer_forward_proj(data,camera_para)
    D = ones(480,640)*inf;
    cnt = 0; %count the nonunique_nonproj points
    for i = 1:size(data,1)
        x = data(i,1); y = data(i,2); z = data(i,3);
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


function D = zbuffer_forward_proj_pointSpliting(pc,camera_para,step)
    D = ones(480,640)*inf;
    cnt = 0; %count the nonunique_nonproj points
    for i = 1:size(pc,1)
        x = pc(i,1); y = pc(i,2); z = pc(i,3);
%         u = round(x/z*camera_para.fx + camera_para.cx + 0.5);
%         v = round(y/z*camera_para.fy + camera_para.cy + 0.5);
        u = round(x/z*camera_para.fx + camera_para.cx);
        v = round(y/z*camera_para.fy + camera_para.cy);
%         u_upper = ceil(u); v_upper = ceil(v);
%         u_lower = floor(u); v_lower = floor(v);
        d = z;
        if u < step+1 || u > 640-step || v < step+1 || v > 480-step, continue; end
        for i = -step:step
            for j = -step:step
                if D(v+j,u+i) > d % the i_th point is not occluded
                    D(v+j,u+i) = d;
                end
            end
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