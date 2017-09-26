%% EVALUATE_RM: Compute RM for GSM dataset
%  Note: the bigger Recall is, the higher rank is;
%        the bigger Specificity is, the higher rank is;         
%        the bigger Precision is, the higher rank is;

% data = [Recall  Specificity  FPR  FNR  PWC/%  F-Measure  Precision]
function evaluate_RM
    data = [0.000 	1.000 	0.000 	0.000 	0.001 	0.000 	0.000 
0	0.997	0.003	0	0.307	0	0
0	0.998	0.002	0	0.187	0	0
0.15	0.48	0	0	0	0	0.52
0.37	0.61	0	0	0	0	0.64
0.49	0.75	0	0	0	0	0.68
];

    RM = computeRM(data);
end

%% COMPUTE_RM: Here only consider dimension 1,3,5,7 in data
function RM = computeRM(data)
    validData = data(:,[1,2,7]);
    rankSet = zeros(size(validData));
    rank    = zeros(size(validData));     %not the original array
    
    [~, rank(:,1)] = sort(validData(:,1),'descend');    
    [~, rank(:,2)] = sort(validData(:,2),'descend');      
    [~, rank(:,3)] = sort(validData(:,3),'descend');          
    
    %%======transform indice into original format=======
    for j = 1:3      %metric
        for i = 1:6  %value of the j_th metric
            rankSet(rank(i,j),j) = i;
        end
    end
    
    %%======compute RM=======
    RM = mean(rankSet,2);
end






















