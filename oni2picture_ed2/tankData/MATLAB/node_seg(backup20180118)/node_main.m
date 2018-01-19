%% NODE_MAIN:use K-means to classify pointcloud
function node_main
k = 15;%depth frame num
D = imread(['E:\dataSet\Wajueji_2\processedData\extractdata_afterDRev\d_',int2str(k),'.png']);

% figure(1),imshow(mat2gray(D)),title('raw D');

%% cluster
cl = 8;
X = getUV(D);
tic;
[idx, ctrs] = kmeans(X, cl);
toc;
figure(50)
plot(X(idx==1,1),X(idx==1,2),'r.','MarkerSize',12);
hold on
plot(X(idx==2,1),X(idx==2,2),'b.','MarkerSize',12);
plot(X(idx==3,1),X(idx==3,2),'g.','MarkerSize',12);
plot(X(idx==4,1),X(idx==4,2),'m.','MarkerSize',12);
plot(X(idx==5,1),X(idx==5,2),'c.','MarkerSize',12);
plot(X(idx==6,1),X(idx==6,2),'y.','MarkerSize',12);
plot(X(idx==7,1),X(idx==7,2),'co','MarkerSize',12);
plot(X(idx==8,1),X(idx==8,2),'yo','MarkerSize',2);
plot(ctrs(:,1),ctrs(:,2),'kx',...
     'MarkerSize',12,'LineWidth',2);
% plot(ctrs(:,1),ctrs(:,2),'ko',...
%      'MarkerSize',12,'LineWidth',2)
legend('Cluster 1','Cluster 2','Cluster 3','Cluster 4','Centroids',...
       'Location','NW')
   xlabel('x'), ylabel('y');
hold off
end

function x = getUV(D)
    x = [];
    [H, W]  = size(D);
    for r = 1:H
        for c = 1:W
            if D(r,c) > 0
                x = [x; [r,c]];
            end
        end
    end
end