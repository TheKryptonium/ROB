%setting frames speed
d=1;
j=1:d:length(T);

%Generating images in 2D
figure
for i=1:length(j)-1
    hold off
    plot3([x1(j(i)) x2(j(i)) x3(j(i))], [y1(j(i)) y2(j(i)) y3(j(i))], [z1 z2 z3(j(i))],'o', ...
        [0 x1(j(i))],[0 y1(j(i))],[0 z1] ,'k', ...
        [x1(j(i)) x2(j(i))], [y1(j(i)) y2(j(i))],[z1 z2], 'k', ...
        [x2(j(i)) x3(j(i))], [y2(j(i)) y3(j(i))],[z2 z3(j(i))], 'k');
    title('Motion of 2DOF Robotic Arm');
    xlabel("X");
    ylabel('Y');
    grid
    hold on;
    MM(i)=getframe(gcf);
end
drawnow;