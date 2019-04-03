%% Clear
clc
clear

%% -
%A=importdata('C:\Users\Halil Temurtaþ\Desktop\Ekstra\8th_Semester\EE494\Step_Responses\deneme.txt')
%T=importdata('C:\Users\Halil Temurtaþ\Desktop\Ekstra\8th_Semester\EE494\Step_Responses\test.txt')
figure()
load 'formodeltest6.txt'
load 'test62.txt'
time=formodeltest6(:,1);
times=time/1000
pixerror=formodeltest6(:,2);
input=test62(:,2);
plot(times,pixerror);
%lettersign=T.textdata(:,2)
%time=T.textdata(:,1)
%abspixerror=T.data
%% -
Go=P3D/(1-P3D)
Go=minreal(Go)
Gp=Go/0.3
Gp=minreal(Gp)
%% -
%lettersign=cell2mat(lettersign)
%time2=cell2mat(time)
%% -
%time=cell2table(time)
%time=table2array(time)
%% -
%arrl=length(lettersign)
%distsign=zeros(1,arrl)
%pixerror=zeros(1,arrl)
%i=1
%while i<arrl+1
%        if (lettersign(i)=='B')
%            pixerror(1,i)=-abspixerror(i);
%            distsign(1,i)=1;
%        elseif (lettersign(i)=='A')
%            distsign(1,i)=0;
%            pixerror(1,i)=abspixerror(i);
%        end
%        
%        i=i+1    
%end


%% -