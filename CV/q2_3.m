clear all
close all

% Loading data
dataDir = 'data';
load(fullfile(dataDir, 'noisy_correspondences.mat'));
img1 = imread(fullfile(dataDir, 'i1.jpg'));
img2 = imread(fullfile(dataDir, 'i2.jpg'));

% Normalization factor (larger dim of the input image
nfactor = max(size(img1, 1), size(img1, 2));

% Compute F
[F7, inliers] = ransacF(pts1, pts2, nfactor)

% Display
displayEpipolarF(img1, img2, F7);
