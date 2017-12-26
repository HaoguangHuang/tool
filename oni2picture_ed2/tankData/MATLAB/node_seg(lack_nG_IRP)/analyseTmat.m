%% analyseTmat: Visualize rotation angle and translation of transformation, then classify them to clusters           
%          Tmat: affine3d. Result of hierarchical node ICP 
function analyseTmat(Tmat)
    if nargin < 1
        tmp = load('E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\MATLAB\node_seg\hhg.mat','Tmat');
        Tmat = tmp.Tmat; %cell
    end
    Tmat_layer4 = Tmat{4};
    R_L4 = cell(1,size(Tmat_layer4,2)); t_L4 = cell(1,size(Tmat_layer4,2));
    for i = 1:size(Tmat_layer4,2)
        R_L4{i} = rodrigues(Tmat_layer4{i}.T(1:3,1:3)); t_L4{i} = Tmat_layer4{i}.T(end,1:3)';
    end
    %%===============visualize Rotation===============%%
    MarkerSpecs = {'d','o','x','+'};
    ColorSpecs = {'r','g','b','k'};
    
    for i = 1:size(Tmat_layer4,2)
        yushu = mod(i,4); shang = floor(i/4);
        figure(51);hold on;
        plot3(R_L4{i}(1),R_L4{i}(2),R_L4{i}(3),[ColorSpecs{shang+1},MarkerSpecs{yushu+1}],'markersize',10);title('Rotation--Layer4');
    end
    legend('1','2','3','4','5','6','7','8','9','10','11'); view([-164,-42]);
    %%===============visualize translation===============%%
    for i = 1:size(Tmat_layer4,2)
        yushu = mod(i,4); shang = floor(i/4);
        figure(52);hold on;
        plot3(t_L4{i}(1),t_L4{i}(2),t_L4{i}(3),[ColorSpecs{shang+1},MarkerSpecs{yushu+1}],'markersize',10);title('translation--Layer4'); 
    end
    legend('1','2','3','4','5','6','7','8','9','10','11'); view([76,45]);
end