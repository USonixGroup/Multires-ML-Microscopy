clear
clc
close all

load("net_checkpoint__200__2024_11_12__23_26_20.mat") %load data from trained network

%%
tiledlayout(2,1, "TileSpacing","tight")
nexttile
plot(vertcat(info.TrainingLoss))
hold on
set(gcf, "color", [1 1 1])

ValLoss=vertcat(info.ValidationLoss);
x=[1:numel(ValLoss)]';
xv=x(~isnan(ValLoss));

plot( x(~isnan(ValLoss)) , ValLoss( ~isnan(ValLoss) )   , LineWidth=2)

legend('Loss', 'Validation Loss', Interpreter='latex')
grid on
box on
xlabel('Iteration', Interpreter='latex')
ylabel('Loss', Interpreter='latex')
fontname("CMU Serif")

title("ResNet50: First Training Run", Interpreter='latex')
%fontsize(24,"points")


nexttile
plot(vertcat(info.TrainingMaskLoss))
hold on
ValLoss=vertcat(info.ValidationMaskLoss);
x=[1:numel(ValLoss)]';
xv=x(~isnan(ValLoss));
plot( x(~isnan(ValLoss)) , ValLoss( ~isnan(ValLoss) )   , LineWidth=2)
%legend('Loss', 'Validation Loss', Interpreter='latex')
grid on
box on
xlabel('Iteration', Interpreter='latex')
ylabel('Mask Loss', Interpreter='latex')
fontname("CMU Serif")

fontsize(16,"points")