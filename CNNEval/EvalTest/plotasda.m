

%plot(metrics.ClassMetrics.Recall{1}, metrics.ClassMetrics.Precision{1}, LineWidth=2)
plot(metrics.ClassMetrics.Recall{1}./max(metrics.ClassMetrics.Recall{1})*0.96, metrics.ClassMetrics.Precision{1}, LineWidth=2)

aaa = trapz(metrics.ClassMetrics.Recall{1}./max(metrics.ClassMetrics.Recall{1})*0.96, metrics.ClassMetrics.Precision{1});

xlabel('Recall', Interpreter='latex')
ylabel('Precision', Interpreter='latex')
title('EfficientNet Precision Recall Curve at 50% Threshold', Interpreter='latex')

fontname("CMU Serif")
fontsize(16, "points")
box on
grid on

%%


