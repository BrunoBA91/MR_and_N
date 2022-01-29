
indexFinal = 300;

Robot= [0 -0.2 0 1;0.4 0 0 1;0 0.2 0 1]';% The Robot icon is a triangle
% subplot(1,2,2)
axis([-3 3 -2 4])
grid on
hold on
title ('Data on World Reference Frame', 'FontWeight','bold','FontSize',16)
for i=1:4 % plotting the 4 Land Marks
scatter(LandMark(1,i),LandMark(2,i),200, 'k');
end
plot (trajec(:,1), trajec(:,2), 'r.','LineWidth',1.5) % Plotting the trajectory
% t = 0: 2*pi/359 : 2*pi;
% P = polar(t, 4.5 * ones(size(t)));% to fix the limits
% subplot(1,2,1)
% set(P, 'Visible', 'off')
% title ('Laser data at Robot Reference Frame','FontWeight','bold','FontSize',16)

for index=1:indexFinal
% subplot(1,2,1)
% dist=lds_dis(index,2:361);
% polar(t, dist, '--g'); % Ploting the laser data wrt Robot frame
% subplot(1,2,2)
ldxplot=ldx(index,:);
ldyplot=ldy(index,:);
% ldxplot(dist<=0.01)=[];
% ldyplot(dist<=0.01)=[];
Laser=scatter(ldxplot, ldyplot); % plotting the land mark seen by the Robot wrt  world reference frame
Robot_tr=transl(trajec(index,1),trajec(index,2),0)*trotz(mod(trajec(index,3)+pi/2,2*pi))*Robot;% moving the robot 
imrob=patch(Robot_tr(1,:), Robot_tr(2,:),'b');
% Covariance=plot_ellipse(pk.signals.values(1:2,1:2,index),[trajec(index,1), trajec(index,2)],'g'); % Plotting the covariance matrix
pause(0.01);
if index~=indexFinal
    delete(Laser)
    delete(Covariance)
    delete (imrob)
end
end
