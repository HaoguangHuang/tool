%% RECTANGLE_WINDOW
% 用来后期对fg/bg segmentation提取到的结果进行处理，剔除更多没用的点

input_file_addr = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/1_200/houqi_v2/';
output_file_addr = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/1_200/houqi_v2/rec_window/';

win_c_start = 74;
win_c_end = 506;
win_r_start = 29;
win_r_end = 393;

file = dir('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/1_200/houqi_v2/*.png');
file_num = size(file,1);

% figure(1),imshow(uint8(ones(480,640,3)*255)),hold on;
% imshow(rectangle('Position',[win_c_start,win_r_start,win_c_end-win_c_start,win_r_end-win_r_start]));
% hold off;
% 
% for i = 36%1:file_num
%     
%     file_name = sprintf('mask_%d.png',i);%file(i).name;
%     frame_no = sscanf(file_name,'mask_%d.png');
%     
%     input_d = imread([input_file_addr,file_name]);
%     output_d = zeros(size(input_d)); 
%     output_d(win_r_start:win_r_end,win_c_start:win_c_end) = input_d(win_r_start:win_r_end,win_c_start:win_c_end);
%     
%     fg = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/color/fusionedForegroundData/fusionedForegroundData%d.png',frame_no));
%     I(:,:,1) = mat2gray(output_d)*255;
%     I(:,:,2) = fg(:,:,1);
%     I(:,:,3) = zeros(size(input_d));
%     
%     
%     imwrite(uint8(mat2gray(output_d)*255),[output_file_addr,file_name]);
%     imwrite(uint8(I),[output_file_addr,'vis/vis_',int2str(frame_no),'.png']);
%     disp(['finish frame_',int2str(frame_no)]);
% end

%======exclude points with value that equals to 1======

% frame_no = 36;
% d = imread([output_file_addr,'mask_',int2str(frame_no),'.png']);
% fg = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/color/fusionedForegroundData/fusionedForegroundData%d.png',frame_no));
% file_name = sprintf('mask_%d.png',frame_no);
% ex_r_s = 282; ex_r_e = 292;
% ex_c_s = 310; ex_c_e = 334;
% 
% I(:,:,1) = mat2gray(d)*255;
% I(:,:,2) = fg(:,:,1);
% I(:,:,3) = zeros(size(d));
%     
% figure(3),imshow(uint8(I)),title('before exclude');
% 
% d(ex_r_s:ex_r_e,ex_c_s:ex_c_e) = 0;
% I(:,:,1) = mat2gray(d)*255;
% 
% figure(4),imshow(uint8(I)),title('after exclude');

% imwrite(uint8(mat2gray(d)*255),[output_file_addr,file_name]);
% imwrite(uint8(I),[output_file_addr,'vis/vis_',int2str(frame_no),'.png']);

%======include points with value that equals to 1======
frame_no = 36;
d = imread([output_file_addr,'mask_',int2str(frame_no),'.png']);
fg = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/color/fusionedForegroundData/fusionedForegroundData%d.png',frame_no));
file_name = sprintf('mask_%d.png',frame_no);
in_r_s = 261; in_r_e = 281;
in_c_s = 314; in_c_e = 320;
I(:,:,1) = mat2gray(d)*255;
I(:,:,2) = fg(:,:,1);
I(:,:,3) = zeros(size(d));
figure(3),imshow(uint8(I)),title('before include');
d(in_r_s:in_r_e,in_c_s:in_c_e) = 255;
I(:,:,1) = mat2gray(d)*255;

figure(4),imshow(uint8(I)),title('after exclude');

% imwrite(uint8(mat2gray(d)*255),[output_file_addr,file_name]);
% imwrite(uint8(I),[output_file_addr,'vis/vis_',int2str(frame_no),'.png']);