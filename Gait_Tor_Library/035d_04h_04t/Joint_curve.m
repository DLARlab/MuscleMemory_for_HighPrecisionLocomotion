an = reshape(gait(1).params.atime,12,6);
ab = reshape(gait(3).params.atime,12,6);
ac = reshape(gait(5).params.atime,12,6);

coeff1 = an (3, 1:6);
s1 = 0:0.01:1;
cjhd1 = YukaiToolkits.bezier(coeff1, s1);

coeff3 = ab (3, 1:6);
s3 = 0:0.01:1;
cjhd3 = YukaiToolkits.bezier(coeff3, s3);

coeff5 = ac (3, 1:6);
s5 = 0:0.01:1;
cjhd5 = YukaiToolkits.bezier(coeff5, s5);

figure;
plot(s1, cjhd1)

figure;
plot(s3, cjhd3)

figure;
plot(s5, cjhd5)