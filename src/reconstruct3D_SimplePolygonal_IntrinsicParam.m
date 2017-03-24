K = [-100 0 200 ; 0 -100 200 ;
0 0 1];
% extrinsic camera parameters
Mextleft = [ 0.707 0.707 0 -3 ;-0.707 0.707 0 -0.5; 0 0 1 3]; Mextright = [ 0.866 -0.5 0 -3 ;0.5 0.866 0 -0.5; 0 0 1 3];
pts = [ 2 0 0 ; 3 0 0;
3 1 0;
2 1 0; 201;
3 0 1;
3 1 1;
2 1 1;
2.5 0.5 2];
NN = 9;
pix = zeros(NN,3); for i = 1:NN,
pixels = K*Mextleft * [pts(i,1) pts(i,2) pts(i,3) 1]'; leftpix(i,:) = pixels./pixels(3);
pixels = K*Mextright * [pts(i,1) pts(i,2) pts(i,3) 1]'; rightpix(i,:) = pixels./pixels(3);
end
% rightpix and leftpix are the list of corresponding points (attainable by % ginput also
figure(1);clf;
drawmyobject(leftpix); title('Left Image'); figure(2);clf;
drawmyobject(rightpix); title('Right Image');

%% Pixel to Rays
rightray = inv(K)*[rightpix(:,1) rightpix(:,2) rightpix(:,3)]'; leftray = inv(K)*[leftpix(:,1) leftpix(:,2) leftpix(:,3)]';

%% STEREO RECONSTRUCTION With known camera matrices
Trw = [Mextright ; 0 0 0 1];
Tlw = [Mextleft; 0 0 0 1];
Twr = inv(Trw); % can be done using transpose Twl = inv(Tlw); % can be done using transpose
Tlr = Tlw*Twr;
% Rotation from right to left coordinate frame
Rlr = Tlr(1:3,1:3);
% translation
tlr = Tlr(1:3,4);
reconpts = reconstruct3d(leftray,rightray,Rlr,tlr,Twl);
figure(3);clf;view(3) drawmy3dobject(pts(:,1:3));title('Original 3D Points');
figure(4);clf;view(3) drawmy3dobject(reconpts(:,1:3));title('Euclidian Reconstruction');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now assume that no calibration parameters are known and only point correspondences % are given
% Reconstruct F from point correspondences. %
% Using the epipolar constraint between the point correspondences, F can be % found
for i = 1:NN
tt=leftpix(i,:)' * rightpix(i,:); % 3 x3 matrix
%form the matrix for Aq = 0, where q is 9x1 the elements of F A(i,:) = [tt(1,:) tt(2,:) tt(3,:)];
end;
[U,S,V] = svd(A);
lastcol = V(:,9);
F(1,1) = lastcol(1); F(1,2) = lastcol(2); F(1,3) = lastcol(3); F(2,1) = lastcol(4); F(2,2) = lastcol(5); F(2,3) = lastcol(6); F(3,1) = lastcol(7); F(3,2) = lastcol(8); F(3,3) = lastcol(9);
% Compare this to the built in call
 compareF = estimateFundamentalMatrix(rightpix(:,1:2),leftpix(:,1:2),'Method','Norm8Point');
fprintf('\n%s\n','The F (from the original code) and F (got from MATLAB built-in funcion) are different')
fprintf('%s %5e %15e %15e\n','F from the original code: ',F(1,1),F(1,2),F(1,3)) fprintf('\t\t\t %5e %15e %15e\n',F(2,1),F(2,2),F(2,3))
fprintf('\t\t\t %5e %15e %15e\n',F(3,1),F(3,2),F(3,3))
fprintf('%s %5e %15e %15e\n','F from the MATLAB built-in function: ',compareF(1,1),compareF(1,2),compareF(1,3))
fprintf('\t\t\t\t %5e %15e %15e\n',compareF(2,1),compareF(2,2),compareF(2,3)) fprintf('\t\t\t\t %5e %15e %15e\n',compareF(3,1),compareF(3,2),compareF(3,3))
% Get Camera Matrix From F
cameramatrix = cammatrix_fromF(F); % WRITE THIS FUNCTION for HW
% Reconstruct with triangulation
Rlr = cameramatrix(1:3,1:3);
tlr = cameramatrix(1:3,4);
prlist = [rightpix(:,1) rightpix(:,2) rightpix(:,3)]';
pllist = [leftpix(:,1) leftpix(:,2) leftpix(:,3)]';
reconpts = reconstruct3d_myfunction(prlist,pllist,Rlr,tlr); % WRITE THIS FOR HW
% Result is up to a projective transformation
figure(5);clf;view(2); drawmy3dobject(reconpts(:,1:3));title('Reconstruction up to a Projective Transformation');
