a =MatlabImportToolpaste2323376023618150602tmp;


%%
plot(a.VarName2./3188*2, a.VarName6)
hold on
plot(a.VarName2./3188*2, movmean(a.VarName6,750), LineWidth=3)

vald = ~isnan(a.VarName11);

%plot(a.VarName2(vald)./3188*2, a.VarName11(vald)-0.5, LineWidth=3)
plot([0:9], [2.871, 1.543, 1.263, 0.6695, 0.632 0.652 0.5796 0.52 0.499 0.4991], LineWidth=3)
legend('Training Loss', '750 Iterations Moving Average', 'Validation Loss', Interpreter='latex')
xlabel('Epoch', Interpreter='latex')
ylabel('Loss', Interpreter='latex')
fontname('CMU Serif')
fontsize(16, 'points')

box on
grid on

%%

