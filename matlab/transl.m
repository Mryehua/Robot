function T=transl(d)
%坐标平移变换
%Ti为被平移的（坐标系）矩阵，d为平移的向量

T=[eye(3),d;0,0,0,1];
end