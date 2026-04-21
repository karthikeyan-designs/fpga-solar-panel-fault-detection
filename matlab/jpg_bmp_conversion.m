clc;
clear;
close all;

% ================================
% INPUT: List of image files
% ================================
imageFiles = {
    'Delamination_solar.jpeg'
};

% ================================
% OUTPUT FOLDER (auto-create)
% ================================
outputFolder = 'Delamination_bmp';

if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
    fprintf('Created folder: %s\n', outputFolder);
end

% ================================
% PROCESS LOOP
% ================================
for i = 1:length(imageFiles)
    
    % Check if file exists
    if ~isfile(imageFiles{i})
        fprintf('File not found: %s\n', imageFiles{i});
        continue;
    end
    
    % Read image
    img = imread(imageFiles{i});
    
    % Convert to grayscale
    if size(img, 3) == 3
        img_gray = rgb2gray(img);
    else
        img_gray = img;
    end
    
    % Resize to 512x512
    img_resized = imresize(img_gray, [512 512]);
    
    % Generate output filename
    [~, name, ~] = fileparts(imageFiles{i});
    outputPath = fullfile(outputFolder, strcat(name, '.bmp'));
    
    % Save image
    imwrite(img_resized, outputPath);
    
    fprintf('Saved: %s\n', outputPath);
end

% ================================
% DONE MESSAGE
% ================================
disp('All images processed successfully!');
