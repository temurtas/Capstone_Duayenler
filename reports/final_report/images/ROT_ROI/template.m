%% Clear
clc
clear

%% -
load 'voltageSupply.txt'
h0=figure('units','normalized','outerposition',[0 0 0.75 0.6])
number=voltageSupply(:,1);
voltage=voltageSupply(:,2);
processTime=voltageSupply(:,3);
yyaxis left
stem(number,voltage,'LineWidth',1.5)
ylim([4.25,5.25])
ylabel('Voltage Supply (V)')
hold on
yyaxis right
plot(number,processTime,'LineWidth',1.5)
ylim([44,150])
ylabel('Processing Time (msec)')
legend('Supply Voltage from Powerbank','Processing Time for Each Frame')
title('Supply Voltage Dependency of Processing Time')
xlabel('Image Processing Iteration')
print(h0,sprintf('voltageSupply.png'),'-dpng','-r600')
close(h0)
%% -
load 'testTime.txt'
h1=figure('units','normalized','outerposition',[0 0 0.75 0.6])
time=testTime(:,1);
times=time/1000
basetimes=times(1)
ProcessTime=testTime(:,2);
sum=0
for i=1:73
   sum = sum+ProcessTime(i)
end
for i=1:73
    times(i)=times(i)-basetimes
end
Avg=sum/73
line=Avg*ones(1,73)
plot(times,ProcessTime,'LineWidth',2);
hold on
plot(times,line,'r','LineWidth',1)
ylim([40 70])
xlim([0 70])
grid on
xlabel('Race Time (s)')
ylabel('Frame Process Time (ms)')
title('Frame Process Time vs Race Time')
legend('Process Time','Average Process Time')
%lettersign=T.textdata(:,2)
%time=T.textdata(:,1)
%abspixerror=T.data
print(h1,sprintf('ProcessTime.png'),'-dpng','-r600')
close(h1)
%% -
h2=figure('units','normalized','outerposition',[0 0 0.45 0.45])
load 'ccw_lapTimes_2.txt'
number=ccw_lapTimes_2(:,1);
LapTime=ccw_lapTimes_2(:,2);
linea=20*ones(1,22)
stem(number,LapTime,'LineWidth',1.5)
hold on
plot(number,linea,'r','LineWidth',1.5)
ylim([8 22])
yticks(8:1:22)
xlim([0 21])
grid on
xlabel('CCW Lap Count')
ylabel('CCW Lap Time (s)')
title('CCW Lap Time vs Lap Count')
legend('CCW Lap Time','Lap Time Limit')
print(h2,sprintf('ccwLapTime.png'),'-dpng','-r600')
close(h2)
%% -
h3=figure('units','normalized','outerposition',[0 0 0.45 0.45])
load 'cw_lapTimes_2.txt'
number=cw_lapTimes_2(:,1);
LapTime=cw_lapTimes_2(:,2);
lineb=20*ones(1,26)
stem(number,LapTime,'LineWidth',1.5)
hold on
plot(number,lineb,'r','LineWidth',1.5)
ylim([8 22])
yticks(8:1:22)
xlim([0 21])
grid on
xlabel('CW Lap Count')
ylabel('CW Lap Time (s)')
title('CW Lap Time vs Lap Count')
legend('CW Lap Time','Lap Time Limit')
print(h3,sprintf('cwLapTime.png'),'-dpng','-r600')
close(h3)