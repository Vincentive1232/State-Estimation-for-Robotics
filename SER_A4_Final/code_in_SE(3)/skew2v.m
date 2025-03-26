% 这个函数是将旋转向量从3*3的反对称矩阵转换到R^3向量空间，方便提取对应的欧拉角
function v = skew2v(M)
    v = 0.5 * [M(3,2)-M(2,3); M(1,3)-M(3,1); M(2,1)-M(1,2)];
end