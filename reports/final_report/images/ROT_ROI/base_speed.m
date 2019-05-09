%% -
clc
clear
%% -
h1=figure('units','normalized','outerposition',[0 0 0.75 0.6])
beta=0:50;
K_max=48;
if (beta>22.5)
    K_min=26;
else
    K_min=28;
end
K_1=0.6;
for i=1:51
    baseSpeed(i)=max(K_max-K_1*beta(i),K_min)
    baseSpeed(i)=fix(baseSpeed(i))
end
plot(beta,baseSpeed,'r','LineWidth',1.5)
ylim([0,50])
grid on
ylabel('Base Speed (PWM)')
xlabel('\beta Angle (Degree)')
title('\beta vs Base Speed')
print(h1,sprintf('baseSpeed.png'),'-dpng','-r600')
close(h1)
%% -
h2=figure('units','normalized','outerposition',[0 0 0.75 0.6])
beta_exp=[20 21.8 32.5 43.6 32.5 21.5 15.7 14.3 10.53 4.5 1.5 3.6 7.8 11.4 16.43 20.42 24.54 32.34 42.4 32.54 21.4 16.4 ]
K_max=38;
K_1=0.7
for i=1:22
    if (beta_exp(i)>22.5)
        K_min=26;
    else
        K_min=28;
    end
    baseSpeed_exp(i)=max(K_max-K_1*beta_exp(i),K_min)
    baseSpeed_exp(i)=fix(baseSpeed_exp(i))
end
num=1:22
yyaxis left
stem(num,beta_exp,'LineWidth',1.5);
% ylim([4.25,5.25])
ylabel('\beta Angle (Degree)')
hold on
yyaxis right
plot(num,baseSpeed_exp,'b','LineWidth',1.5)
ylim([0,50])
ylabel('Base Speed (PWM)')
legend('Supply Voltage from Powerbank','Processing Time for Each Frame')
title('Experimental \beta vs Base Speed')
xlabel('Image Processing Iteration')
print(h2,sprintf('baseSpeedExp.png'),'-dpng','-r600')
close(h2)