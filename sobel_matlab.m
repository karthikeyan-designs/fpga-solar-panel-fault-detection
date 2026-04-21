clc;
clear;
close all;

% Read grayscale image
img = imread('sat.bmp');

% Ensure uint8
img = uint8(img);

% Convert to double for processing
img_d = double(img);

% Sobel Kernels (same as your Verilog)
Gx = [ 1  0 -1;
       2  0 -2;
       1  0 -1];

Gy = [ 1  2  1;
       0  0  0;
      -1 -2 -1];

% Convolution
Ix = conv2(img_d, Gx, 'same');
Iy = conv2(img_d, Gy, 'same');

% Magnitude (like absX + absY in your RTL)
magnitude = abs(Ix) + abs(Iy);

% Threshold (adjust if needed)
threshold = 70;
edges = magnitude > threshold;

% Convert to uint8 (0 or 255)
edges = uint8(edges) * 255;

% Display
figure;
subplot(1,2,1);
imshow(img);
title('Input Image');

subplot(1,2,2);
imshow(edges);
title('Sobel Edge Output');

% Save result
imwrite(edges,'sobel_matlab_output.bmp');