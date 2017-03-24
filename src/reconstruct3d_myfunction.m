function reconstructed = reconstruct3d_myfunction(prlist,pllist,Rlr,tlr) for i = 1:3
cameraMatrixfromF(i,:) = [Rlr(i,:) tlr(i)]; end;
for i = 1:2
pr2d(i,:) = prlist(i,:)./prlist(3,:); pl2d(i,:) = pllist(i,:)./pllist(3,:);
    end;
    pr2d = pr2d';
pl2d = pl2d';
identityMatrix = [1 0 0 0; 0 1 0 0;
0 0 1 0];
reconstructed = triangulate(pr2d,pl2d,identityMatrix',cameraMatrixfromF');