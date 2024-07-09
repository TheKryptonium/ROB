clc; close all;

%setting frames speed
d=5;
j=1:d:length(T);

%Generating images in 2D
figure(1);
for i=1:length(j)-1
    hold off
    p3=plot3([x0 x1(j(i)) x2(j(i)) x3(j(i))], [y0 y1(j(i)) y2(j(i)) y3(j(i))], [z0 z1 z2 z3(j(i))],'o',...
        [0 x0],[0 y0],[0 z0],'k', ...
        [x0  x1(j(i))],[y0 y1(j(i))],[z0 z1],'k',...
        [x1(j(i)) x2(j(i))], [y1(j(i)) y2(j(i))],[z1 z2], 'k', ...
        [x2(j(i)) x3(j(i))], [y2(j(i)) y3(j(i))],[z2 z3(j(i))], 'k');
    title('Motion of 3DOF Robotic Arm');
    xlabel("X");
    ylabel('Y');
    zlabel("Z");
    axis([0 1 0 1 0 0.5]);
    grid
    hold on;
    MM(i)=getframe(gcf);
end
drawnow;