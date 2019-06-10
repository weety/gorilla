% Generate fuzzy control lookup table
% Author: weety
% 2019/06/08

clear all;
fuzzy_control = readfis('gorilla_fuzzy')

fuzzy_table = zeros(13,13,2);
for i = -6:6
    for j = -6:6
        fuzzy_table(i+7,j+7,:) = evalfis([i,j], fuzzy_control)
    end
end

%fuzzy_table_csv = 'gorilla_fuzzy_table.csv';
%fid = fopen(fuzzy_table_csv, 'w');
%����KPģ����ѯ��
%ת��
m1=single(fuzzy_table(:,:,1)');
%��ͷ
various={'E_EC','N6','N5','N4','N3','N2','N1','Z0','P1','P2','P3','P4','P5','P6'};

cloums={'N6';'N5';'N4';'N3';'N2';'N1';'Z0';'P1';'P2';'P3';'P4';'P5';'P6'};
%�������
kp_table=table(cloums,m1(:,1),m1(:,2),m1(:,3),m1(:,4),m1(:,5),m1(:,6),m1(:,7),m1(:,8),m1(:,9),m1(:,10),m1(:,11),m1(:,12),m1(:,13),'VariableNames',various);
%����csv���
writetable(kp_table, 'gorilla_fuzzy_kp_table.csv')

%����KDģ����ѯ��
%ת��
m1=single(fuzzy_table(:,:,2)');
%��ͷ
various={'E_EC','N6','N5','N4','N3','N2','N1','Z0','P1','P2','P3','P4','P5','P6'};

cloums={'N6';'N5';'N4';'N3';'N2';'N1';'Z0';'P1';'P2';'P3';'P4';'P5';'P6'};
%�������
kd_table=table(cloums,m1(:,1),m1(:,2),m1(:,3),m1(:,4),m1(:,5),m1(:,6),m1(:,7),m1(:,8),m1(:,9),m1(:,10),m1(:,11),m1(:,12),m1(:,13),'VariableNames',various);
%����csv���
writetable(kd_table, 'gorilla_fuzzy_kd_table.csv')