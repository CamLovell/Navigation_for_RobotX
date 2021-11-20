% This function relies or saved data being loaded in
% See "../Data/Saved Runs"

avgRMS = mean(RMS(2:end-1));
maxE = max(abs(x(2,5:end)-xMLHist(2,5:end)));
maxN = max(abs(x(1,5:end)-xMLHist(1,5:end)));
maxPsi = max(abs(x(3,5:end-1)-xMLHist(3,5:end-1)))*180/pi;

% Plotting Tracking Comparison
figure(1);
%Plot North Position
subplot(2,2,1);
plot(tHist,x(1,:),tHist,xMLHist(1,:)); 
legend("Actual","Estimated");
title("North Position");
xlabel("Time (s)");
ylabel("North (m) ");
%Plot East Position
subplot(2,2,2);
plot(tHist,x(2,:),tHist,xMLHist(2,:));
legend("Actual","Estimated");
title("East Position");
xlabel("Time (s)");
ylabel("East (m)");
%Plot Psi Angle
subplot(2,2,3);
plot(tHist,x(3,:)*180/pi,tHist,xMLHist(3,:)*180/pi);
legend("Actual","Estimated");
title("Heading Angle");
xlabel("Time (s)");
ylabel("Heading Angle (\circ)");
%Plot Psi Angle
subplot(2,2,4);
plot(x(2,:),x(1,:),xMLHist(2,:),xMLHist(1,:));
legend("Actual","Estimated");
title("N-E Combined 2D");
xlabel("East (m)");
ylabel("North (m)");
sgtitle("Estimated vs Actual States");

% Plot Tracking Errors
figure(2);
%Plot North Error
subplot(2,2,1);
plot(tHist,x(1,:)-xMLHist(1,:));
title("Absolute Error in N");
xlabel("Time (s)");
ylabel("Error (m) ");
%Plot East Error
subplot(2,2,2);
plot(tHist,x(2,:)-xMLHist(2,:));
title("Absolute Error in E");
xlabel("Time (s)");
ylabel("Error (m)");
%Plot Psi Angle Error
subplot(2,2,3);
plot(tHist,x(3,:)*180/pi-xMLHist(3,:)*180/pi);
title("Absolute Error in \psi");
xlabel("Time (s)");
ylabel("Error (\circ)");
%Plot RMS position error
RMS = sqrt(((x(1,:)-xMLHist(1,:)).^2+(x(2,:)-xMLHist(2,:)).^2)/2);
subplot(2,2,4);
plot(tHist,RMS(:));
title("RMS Position Error");
xlabel("Time (s)");
ylabel("Error (m)");
sgtitle("Error in States");