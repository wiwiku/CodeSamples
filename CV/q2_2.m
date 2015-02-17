clear all
close all

% Loading data
dataDir = 'data';
load(fullfile(dataDir, 'clean_correspondences.mat'));
img1 = imread(fullfile(dataDir, 'i1.jpg'));
img2 = imread(fullfile(dataDir, 'i2.jpg'));

% Normalization factor (larger dim of the input image
nfactor = max(size(img1, 1), size(img1, 2));

% Compute F7 using clean correspondences
F7 = sevenpoint_norm(pts1(:, 1:7), pts2(:, 1:7), nfactor)

x = [pts1(:, 1); 1]; xp = [pts2(:, 1); 1];
error = xp' * F7{1} * x

% Display
displayEpipolarF(img1, img2, F7{1});