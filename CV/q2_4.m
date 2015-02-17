clear all
close all
addpath(genpath('.'));

tic


% Loading data
dataDir = 'data';
load(fullfile(dataDir, 'noisy_correspondences.mat')); % Correspondences
load(fullfile(dataDir, 'K.mat')); % Instrinsic matrix K
img1 = imread(fullfile(dataDir, 'i1.jpg'));
img2 = imread(fullfile(dataDir, 'i2.jpg'));

% Normalization factor: larger dim of the input image
nfactor = max(size(img1, 1), size(img1, 2));

% Compute F and inliers
[F7, inliers] = ransacF(pts1, pts2, nfactor)
inpts1 = pts1(:, inliers);
inpts2 = pts2(:, inliers);

% Compute camera matrices for camera 1 and camera 2
M1 = K * [eye(3,3), zeros(3,1)];
M2 = camera2(F7, K, K, inpts1, inpts2);

% RANSAC to find H and inliers
[H1, inliersH1] = ransacH(inpts1, inpts2); % Plane 1
inpts1_H1 = inpts1(:, inliersH1);
inpts2_H1 = inpts2(:, inliersH1);
inpts1(:, inliersH1) = []; % Remove inliers
inpts2(:, inliersH1) = []; % Remove inliers

[H2, inliersH2] = ransacH(inpts1, inpts2); % Plane 2
inpts1_H2 = inpts1(:, inliersH2);
inpts2_H2 = inpts2(:, inliersH2);
inpts1(:, inliersH2) = []; % Remove inliers
inpts2(:, inliersH2) = []; % Remove inliers

% Find planes that fit inliers
P1 = triangulate(M1, inpts1_H1, M2, inpts2_H1); % Plane 1
P2 = triangulate(M1, inpts1_H2, M2, inpts2_H2); % Plane 2

% Calculate plane parameters
p1coeff = getPlane(P1);
p2coeff = getPlane(P2);

% Display with different M
R1 = [1 0 0;...
      0 1 0;...
      0 0 1;];
t1 = [0.5 0 0]';
Mc1 = K * [R1 t1];
frame1 = drawNovelView(p1coeff, p2coeff, Mc1);
%imshow(frame1);

d = 35/180*pi;
R2 = [1 0 0;...
      0 cos(d) -sin(d);...
      0 sin(d) cos(d)];
t2 = [0 1.7 0]';
Mc2 = K * [R2, t2];
frame2 = drawNovelView(p1coeff, p2coeff, Mc2);
%imshow(frame2);

%fprintf('Done!\n');

toc
