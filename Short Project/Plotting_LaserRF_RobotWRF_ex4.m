%% Declaration and setup
%x = inputdlg('Enter step time to visualize',... %Introducing the snapshot to visualize
%             'Input', [1 20]);
%index = str2num(x{:});
inici=50;
final=300;
distTH=0.5;

Robot= [0 -0.2 0 1;0.4 0 0 1;0 0.2 0 1]';% The Robot icon is a triangle
% subplot(1,2,2)
axis([-3 3 -2 4])
grid on
hold on
title ('Data on World Reference Frame', 'FontWeight','bold','FontSize',16)
color=[[0,1,0];[0,0.3,1];[1,0,1];[0,0,0]];
for i=1:4
scatter(LandMark(1,i),LandMark(2,i),200,color(i,:));
end

plot (trajec(:,1), trajec(:,2), 'r.','LineWidth',1.5) % Plotting the trajectory
% t = 0: 2*pi/359 : 2*pi;
% P = polar(t, 4.5 * ones(size(t)));% to fix the limits
% subplot(1,2,1)
% set(P, 'Visible', 'off')
% title ('Laser data at Robot Reference Frame','FontWeight','bold','FontSize',16)
correcx=0;
correcy=0;
correcalpha=0;

%% Loop start
for index=inici:final
% subplot(1,2,1)
dist=lds_dis(index,2:361);
% polar(t, dist, '--g'); % Ploting the laser data wrt Robot frame
% subplot(1,2,2)
ldxplot=ldx(index,:)-(correcx*cos(alpha_ST)+correcy*sin(alpha_ST));
ldyplot=ldy(index,:)-(-correcx*sin(alpha_ST)+correcy*cos(alpha_ST));
newLm=[];
LMOK=ones(4,1);
ldxplot(dist<=0.1)=[]; %Eliminate those plots in the robot
ldyplot(dist<=0.1)=[];
for i=1:4 %Group the laser if they are near or far from the LM
   distLM=sqrt((ldxplot-LandMark(1,i)).^2+(ldyplot-LandMark(2,i)).^2);
   goodL=[ldxplot(distLM<distTH);ldyplot(distLM<distTH)];
   if length(goodL)>2
       newLm=[newLm mean(goodL')'];
   else
       LMOK(i)=0;
   end
   LaserLM(i)=scatter(ldxplot(distLM<distTH), ldyplot(distLM<distTH),40,color(i,:));
   ldxplot(distLM<distTH)=[];
   ldyplot(distLM<distTH)=[];
end
LaserLM(5)=scatter(ldxplot, ldyplot,80,'r'); % plotting the land mark seen by the Robot wrt  world reference frame
%% Similarity Transform

Robot_tr=transl(trajec(index,1)-correcx,trajec(index,2)-correcy,0)*trotz(mod(trajec(index,3)+pi/2,2*pi))*Robot;% moving the robot 

if length(newLm)>2
    [tx_ST,ty_ST,alpha_ST]=ST(LandMark(:,LMOK==1),newLm);
    correcx=correcx+tx_ST;
    correcy=correcy+ty_ST;
    correcalpha=correcalpha+alpha_ST;
end

imrob=patch(Robot_tr(1,:), Robot_tr(2,:),'b');
Covariance=plot_ellipse(pk.signals.values(1:2,1:2,index),[trajec(index,1), trajec(index,2)],'g'); % Plotting the covariance matrix
pause(0.01);
if index~=final
delete(LaserLM)
delete(Covariance)
delete (imrob)
end
end
