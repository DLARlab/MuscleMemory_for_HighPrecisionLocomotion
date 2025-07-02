clear all

for i = -10:10
   % aa = sprintf('%d',r);

num = i/10;
formatSpec = '%.1f';
order = num2str(num,formatSpec)
load(['H_0.34_V_' order '_FRFRRRRLlo_.mat'])
stand=gait(1).params.atime;
air=gait(3).params.atime;
save(['Stand_H_0.34_V_' order '_FRFRRRRLlo_.txt'],'stand', '-ASCII')
save(['Air_H_0.34_V_' order '_FRFRRRRLlo_.txt'],'air', '-ASCII')

end
