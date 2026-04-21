clc;
clear;
close all;

% ================================
% IMAGE LIST (direct files)
% ================================
imageFiles = {
    'real_dust_1.jpeg'
    'real_dust_2.jpeg'
    
};

% ================================
% PROCESS LOOP
% ================================
for i = 1:length(imageFiles)

    % Check file exists
    if ~isfile(imageFiles{i})
        fprintf('File not found: %s\n', imageFiles{i});
        continue;
    end

    % Read image
    img = imread(imageFiles{i});

    % Resize
    img_resized = imresize(img, [512 512]);

    % Convert to grayscale
    if size(img_resized,3) == 3
        img_gray = rgb2gray(img_resized);
    else
        img_gray = img_resized;
    end

    % Output filename (same name)
    [~, name, ~] = fileparts(imageFiles{i});
    outputName = strcat(name, '.bmp');

    % Save image
    imwrite(img_gray, outputName);

    fprintf('Saved: %s\n', outputName);
end

disp('All images processed successfully!');
