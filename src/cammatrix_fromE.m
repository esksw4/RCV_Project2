function cammatrixfromE = cammatrix_fromE(matrixE)
W = Z =
[0 -1 0;
 1 0 0;
 0 0 1];
[0 1 0; -1 0 0;
0 0 0];
[U,S,V] = svd(matrixE);
S= U*Z*U';
R = U*W'*V';
t = U(:,3);
for i = 1:3
leftCamTransMatrix(i,:) = [R(i,:) t(i)];
end;
cammatrixfromE = leftCamTransMatrix;