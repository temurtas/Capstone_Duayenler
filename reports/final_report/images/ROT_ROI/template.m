%% Clear
clc
clear

%% -
load 'testTime.txt'
h1=figure('units','normalized','outerposition',[0 0 1 1])
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
h2=figure('units','normalized','outerposition',[0 0 1 1])
load 'ccw_lapTimes.txt'
number=ccw_lapTimes(:,1);
LapTime=ccw_lapTimes(:,2);
linea=20*ones(1,20)
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
h3=figure('units','normalized','outerposition',[0 0 1 1])
load 'cw_lapTimes.txt'
number=cw_lapTimes(:,1);
LapTime=cw_lapTimes(:,2);
lineb=20*ones(1,20)
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