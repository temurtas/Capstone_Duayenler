%% ROI_high
h1=figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,2,1)
ROI_high_x=[0 3 13 15:18]
ROI_high_y=[55 88 255 255 255 255 255]
plot(ROI_high_x,ROI_high_y,'r','LineWidth',2)
xlabel('Sensor Distance (cm)')
ylabel('ROI High (Px)')
ylim([0 300])
title('Region of Interest High vs Sensor Distance')
grid on
%print(h1,sprintf('ROI_HIGH.png'),'-dpng','-r600')
%close(h1)
%% ROT_high
%h2=figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,2,2)
ROT_high_x=[0 3 13 15:18]
ROT_high_y=[54 75 175 175 175 175 175]
plot(ROT_high_x,ROT_high_y,'b','LineWidth',2)
xlabel('Sensor Distance (cm)')
ylabel('ROT High (Px)')
ylim([0 225])
title('Region of Target High vs Sensor Distance')
grid on
%print(h2,sprintf('ROT_HIGH.png'),'-dpng','-r600')
%close(h2)
%% ROI
subplot(2,2,3)
%h3=figure('units','normalized','outerposition',[0 0 1 1])
ROI_x=[1.75 3.51 11.7 12:15]
ROI_y=[30 50 200 200 200 200 200]
plot(ROI_x,ROI_y,'r','LineWidth',2)
xlabel('Coverage (cm)')
ylabel('ROI (Px)')
ylim([0 250])
title('Region of Interest vs Coverage')
grid on
%print(h3,sprintf('ROI.png'),'-dpng','-r600')
%close(h3)
%% ROT
%h4=figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,2,4)
ROT_x=[0.2 1.4 7.3 8:10]
ROT_y=[4 25 125 125 125 125]
plot(ROT_x,ROT_y,'b','LineWidth',2)
xlabel('Coverage (cm)')
ylabel('ROT (Px)')
ylim([0 175])
title('Region of Target vs Coverage')
grid on
%print(h4,sprintf('ROT.png'),'-dpng','-r600')
%close(h4)
print(h1,sprintf('ROI_ROT.png'),'-dpng','-r600')
close(h1)