clear
clc
close all
load("all_bbox_data.mat")

dims = bbox(:, 3:4);

%%
% k-means clustering to find optimal anchor box setup at the given
% magnification
k=9;
[idx centroids] = kmeans(dims, k, 'Replicates',10,'MaxIter',5000);
anchor_widths = centroids(:,1);
anchor_heights = centroids(:,2);

%%
% Calculate aspect ratios
aspect_ratios = anchor_widths ./ anchor_heights;

% Sort by area for better interpretation
areas = anchor_widths .* anchor_heights;
[~, area_idx] = sort(areas);
sorted_anchors = [anchor_widths(area_idx), anchor_heights(area_idx), aspect_ratios(area_idx)];

% Display results
disp('Optimized anchors (width, height, aspect ratio):');
disp(sorted_anchors);

% Visualize anchor boxes
figure;
for i = 1:k
    rectangle('Position', [0, 0, anchor_widths(i), anchor_heights(i)], ...
              'EdgeColor', rand(1,3));
    hold on;
end
axis equal;
title('Optimized Anchor Boxes');

%%
% Calculate IoU between ground truth boxes and optimized anchors
max_ious = zeros(size(dims, 1), 1);

for i = 1:size(dims, 1)
    gt_width = dims(i, 1);
    gt_height = dims(i, 2);
    
    for j = 1:k
        % Calculate IoU between ground truth and anchor
        % (simplified version - assumes boxes are centered)
        min_w = min(gt_width, anchor_widths(j));
        min_h = min(gt_height, anchor_heights(j));
        
        intersection = min_w * min_h;
        union = (gt_width * gt_height) + (anchor_widths(j) * anchor_heights(j)) - intersection;
        
        iou = intersection / union;
        max_ious(i) = max(max_ious(i), iou);
    end
end

% Show statistics
mean_iou = mean(max_ious);
disp(['Mean max IoU: ', num2str(mean_iou)]);
histogram(max_ious);
title('Max IoU Distribution');