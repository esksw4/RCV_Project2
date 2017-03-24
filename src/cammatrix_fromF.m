function camMatrixF = cammatrix_fromF(matrixF) % crate
[m,n] = size(matrixF); identityMatrix = eye(m,n);
% getting epipole_left
[U,S,V] = svd(matrixF);
epipoleleft = V(:,m);
epipolecrossProduct = [0 -epipoleleft(3,1) epipoleleft(2,1);
epipoleleft(3,1) 0 -epipoleleft(1,1);
-epipoleleft(2,1) epipoleleft(1,1) 0]; exF = cross(epipolecrossProduct,matrixF);
for i = 1:m
rightCamMatrix(i,:) = [identityMatrix(i,:) 0]; leftCamMatrix(i,:) = [exF(i,:) epipoleleft(i,1)];
end;
camMatrixF = leftCamMatrix;