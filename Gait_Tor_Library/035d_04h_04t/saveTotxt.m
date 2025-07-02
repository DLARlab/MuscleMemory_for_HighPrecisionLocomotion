clear all

load('035d_04h_04t.mat')
standOff=gait(1).params.atime;
air=gait(3).params.atime;
standTouch=gait(5).params.atime;
save('standOff.txt','standOff', '-ASCII')
save('air.txt','air', '-ASCII')
save('standTouch.txt','standTouch', '-ASCII')
