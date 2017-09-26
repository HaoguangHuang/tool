%% EVALUATE_RC: compute RC for GSM dataset

data = [2.25
5
4.5
5.5
6.25
4.75
3.25
4.5
];

[~, indice] = sort(data,'ascend');
rankSet = zeros(8,1);
for i = 1:8
    rankSet(indice(i),1) = i;
end


data;