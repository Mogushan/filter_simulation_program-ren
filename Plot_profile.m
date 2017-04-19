function Plot_profile(profile)
% Plots navigation solution errors
%
% NEEDS TO BE CORRECTED
%
% created 16/03/17 by Ren
%
% Input:
% errors    Array of error data to plot
% Format is
% Column 1: roll component of ENU attitude error (deg)
% Column 2: pitch component of ENU attitude error (deg)
% Column 3: yaw component of ENU attitude error (deg)
% Column 4: east velocity error(m/s)
% Column 5: north velocity error(m/s)
% Column 6: up velocity error(m/s)
% Column 7: east position error (m)
% Column 8: north position error (m)
% Column 9: up position error (m)
% Column 10: time(s)

%Constants
R = 6378245;

% Begins
fig = figure;
set(fig,'units','normalized');
set(fig,'OuterPosition',[0.5,0.4,0.45,0.6]);

subplot(3,3,1);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0.9,0.45,0]);
plot(profile(:,10),R*profile(:,7),'LineWidth',1.5);
title('North position, (^o)');
set(gca,'OuterPosition',[0.01,0.68,0.32,0.31]);
subplot(3,3,2);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0,0.9,0.45]);
plot(profile(:,10),R*profile(:,8),'LineWidth',1.5);
title('East position,  (^o)');
set(gca,'OuterPosition',[0.34,0.68,0.32,0.31]);
subplot(3,3,3);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0.45,0,0.9]);
plot(profile(:,10),profile(:,9),'LineWidth',1.5);
title('Down position, m');
set(gca,'OuterPosition',[0.67,0.68,0.32,0.31]);

subplot(3,3,4);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0.9,0,0.45]);
plot(profile(:,10),profile(:,4),'LineWidth',1.5);
title('North velocity, m/s');
set(gca,'OuterPosition',[0.01,0.36,0.32,0.31]);
subplot(3,3,5);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0.45,0.9,0]);
plot(profile(:,10),profile(:,5),'LineWidth',1.5);
title('East velocity, m/s');
set(gca,'OuterPosition',[0.34,0.36,0.32,0.31]);
subplot(3,3,6);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0,0.45,0.9]);
plot(profile(:,10),profile(:,6),'LineWidth',1.5);
title('Down velocity, m/s');
set(gca,'OuterPosition',[0.67,0.36,0.32,0.31]);

subplot(3,3,7);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0,0.7,0.7]);
plot(profile(:,10),radtodeg(profile(:,1)),'LineWidth',1.5);
xlabel('Time, s');
title('Attitude about North, deg');
set(gca,'OuterPosition',[0.01,0,0.32,0.35]);
subplot(3,3,8);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0.7,0,0.7]);
plot(profile(:,10),radtodeg(profile(:,2)),'LineWidth',1.5);
xlabel('Time, s');
title('Attitude about East, deg');
set(gca,'OuterPosition',[0.34,0,0.32,0.35]);
subplot(3,3,9);
set(gca,'NextPlot','replacechildren');
set(gca,'ColorOrder',[0.7,0.7,0]);
plot(profile(:,10),radtodeg(profile(:,3)),'LineWidth',1.5);
xlabel('Time, s');
title('Heading, deg');
set(gca,'OuterPosition',[0.67,0,0.32,0.35]);

% Ends