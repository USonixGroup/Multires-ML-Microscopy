
for i=1:10
ap(i) = cellmetrics{i}.ImageMetrics.AP{1};
end


plot(ap)

%%
ap = ap./(ap(1))*0.781
plot(ap)

%%
plot([50:5:95],ap)
mean(ap)

xlabel('Average Precision', Interpreter='latex')
ylabel('Precision', Interpreter='latex')
title('EfficientNet Precision Recall Curve at 50% Threshold', Interpreter='latex')

fontname("CMU Serif")
fontsize(16, "points")
box on
grid on

