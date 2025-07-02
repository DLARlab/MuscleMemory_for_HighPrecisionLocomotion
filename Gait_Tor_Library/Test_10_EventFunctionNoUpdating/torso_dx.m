fly_x=gait(1).states.dx(1,:);
stance_dx=gait(3).states.dx(1,:);

dx = [fly_x,stance_dx];


t=0:1/31:1;
figure;
plot(t,dx)
