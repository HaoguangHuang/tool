%% EVALUATE_RM: Compute RM for Hybrid dataset
%  Note: the smaller TE is, the higher rank is;
%        the smaller FNR is, the higher rank is;         
%        the smaller FPR is, the higher rank is;
%        the bigger S is, the higher rank is;

%data = [TE_Avg/%	TE_Std	FNR_Avg/%	FNR_Std	FPR_Avg/%	FPR_Std	S_Avg	S_Std]
function evaluate_RM
    data = [0.08 	0.05 	3.22 	5.60 	0.03 	0.03 	0.92 	0.16 
3.2	2.77	3.52	0.09	2.92	0.1	0.89	0.15
3.49	3.4	0.38	0.02	6.13	0.14	0.91	0.09
6.49	5.6	0.25	0.01	11.8	0.23	0.84	0.16
6.94	4.13	0.17	0.01	12.69	0.17	0.81	0.18
2.3	2.26	7.1	14.5	3.21	6.3	0.9	0.15
2.2	2.27	2.94	5.53	4.36	6.42	0.92	0.08
2.72	0	2.99	0	1.55	0	0.83	0
];

    RM = computeRM(data);
end

%% COMPUTE_RM: Here only consider dimension 1,3,5,7 in data
function RM = computeRM(data)
    validData = data(:,[1,3,5,7]);
    rankSet = zeros(size(validData));
    rank    = zeros(size(validData));     %not the original array
    
    [~, rank(:,1)] = sort(validData(:,1),'ascend');    
    [~, rank(:,2)] = sort(validData(:,2),'ascend');      
    [~, rank(:,3)] = sort(validData(:,3),'ascend');      
    [~, rank(:,4)] = sort(validData(:,4),'descend');     
    
    %%======transform indice into original format=======
    for j = 1:4      %metric
        for i = 1:8  %value of the j_th metric
            rankSet(rank(i,j),j) = i;
        end
    end
    
    %%======compute RM=======
    RM = mean(rankSet,2);
end






















