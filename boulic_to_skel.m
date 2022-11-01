clc; clear; close all;

load('boulic_data.mat');
nt = 2048; % number of frames per cycle
num_pulse = size(segment(1).PositionData{1}, 2);
duration = num_pulse / nt;

names = cell(1, 1);
skel_hist = zeros(length(segment), 3, num_pulse);

for i = 1:length(segment)
    skel_hist(i, :, :) = segment(i).PositionData{1};
    names{i} = segment(i).name;
end

save('boulic_data_skel.mat', 'skel_hist', 'names');








