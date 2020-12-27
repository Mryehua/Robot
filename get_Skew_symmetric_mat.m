function Ra=get_Skew_symmetric_mat(a)
% 获得向量a对应的斜对称矩阵
Ra=[0,-a(3),a(2);
    a(3),0,-a(1);
    -a(2),a(1),0];
end