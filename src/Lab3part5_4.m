clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);
SERV_ID = 1848;
model = model();
traj = Traj_planner();

%points
vertex1 = [-29.52,70.3,-36.61]; %Position 1
point1 = [154.3290, 87.3863, 73.2396];
point2 = [115.3207,48.4764, 222.5871];
point3 = [70.2094, 142.9362, 70.0196];
%array of vertex in the task space
viaPts = [point1;point2;point3;point1];
%send the robot to the first vertex
packet = pp.setSetpoints(vertex1);
pp.write(SERV_ID, packet);
pause(1);

tic;
if 1
    try
        for i = 1:1:length(viaPts)
            current_pos = pp.getPosition();
            x1Traj = traj.quintic_traj(0,1,0,0,current_pos(1),viaPts(i,1),0,0);
            y2Traj = traj.quintic_traj(0,1,0,0,current_pos(2),viaPts(i,2),0,0);
            z3Traj = traj.quintic_traj(0,1,0,0,current_pos(3),viaPts(i,3),0,0);
            
            
            start = toc;
            delta = toc - start;
            while delta < 1
                delta = toc - start;
                pos_x = x1Traj(1) + x1Traj(2) *(delta) + x1Traj(3) * ((delta)^2) + x1Traj(4) * ((delta)^3) + x1Traj(5) * ((delta)^4) + x1Traj(6) * ((delta)^5);
                pos_y = y2Traj(1) + y2Traj(2) *(delta) + y2Traj(3) * ((delta)^2) + y2Traj(4) * ((delta)^3) + y2Traj(5) * ((delta)^4) + y2Traj(6) * ((delta)^5);
                pos_z = z3Traj(1) + z3Traj(2) *(delta) + z3Traj(3) * ((delta)^2) + z3Traj(4) * ((delta)^3) + z3Traj(5) * ((delta)^4) + z3Traj(6) * ((delta)^5);
                
                task_space_point = double([pos_x,pos_y,pos_z]);
                joint_space_point = pp.ik3001(task_space_point);
                
                packet = pp.setSetpoints(joint_space_point);
                pp.write(SERV_ID, packet);
                
                positions = pp.getPosition();
                task_space_pos = pp.fk3001(positions);
                pos_end = [task_space_pos(1),task_space_pos(2),task_space_pos(3),toc];
                writematrix(pos_end, 'Part5_4.csv', "WriteMode", "append");
                
            end
            start = toc;
            delta = toc - start;
            while delta < .5
                delta = toc - start;
            end
        end
    catch
        pp.shutdown
    end
end
pp.shutdown();

%make the 3d graph
load Part5_4.csv;
xPos = Part5_4(:,1);
yPos = Part5_4(:,2);
zPos = Part5_4(:,3);

figure(1)
hold on;
grid on;
plot3(xPos,yPos,zPos,'r');
scatter3(point1(1),point1(2),point1(3), 'b');
scatter3(point2(1),point2(2),point2(3), 'b');
scatter3(point3(1),point3(2),point3(3), 'b');
title("Tip Positions in task spave");
ylabel("Position Y (mm)");
xlabel("Position X (mm)");
zlabel("Position Z (mm)");
hold off;

%make the pos graph
load Part5_4.csv;
xPos = Part5_4(:,1);
yPos = Part5_4(:,2);
zPos = Part5_4(:,3);
time = Part5_4(:,4);
figure(2)
hold on;
grid on;
plot(time, xPos, "-", "LineWidth", 2);
plot(time, yPos, "-.", "LineWidth", 2);
plot(time, zPos, "--", "LineWidth", 2);
title("Tip Positions over Time");
ylabel("Tip Position (mm)");
xlabel("Time (s)");
legend("X Position", "Y Position", "Z Position");
hold off;

%velocity
load Part5_4.csv;
xPos = Part5_4(:,1);
yPos = Part5_4(:,2);
zPos = Part5_4(:,3);
time = Part5_4(:,4);

xVel = gradient(xPos) ./ gradient(time);
yVel = gradient(yPos) ./ gradient(time);
zVel = gradient(zPos) ./ gradient(time);

figure(3)

hold on;
grid on;
plot(time, xVel, "-", "LineWidth", 2);
plot(time, yVel, "-.", "LineWidth", 2);
plot(time, zVel, "--", "LineWidth", 2);
title("Tip Velocity over Time");
ylabel("Tip Velocity (mm/s)");
xlabel("Time (s)");
legend("X Position", "Y Position", "Z Position");
hold off;

%Aceleration
load Part5_4.csv;
xPos = Part5_4(:,1);
yPos = Part5_4(:,2);
zPos = Part5_4(:,3);
time = Part5_4(:,4);

xVel = gradient(xPos) ./ gradient(time);
yVel = gradient(yPos) ./ gradient(time);
zVel = gradient(zPos) ./ gradient(time);

xAcl = gradient(xVel) ./ gradient(time);
yAcl = gradient(yVel) ./ gradient(time);
zAcl = gradient(zVel) ./ gradient(time);

figure(4)

hold on;
grid on;
plot(time, xAcl, "-", "LineWidth", 2);
plot(time, yAcl, "-.", "LineWidth", 2);
plot(time, zAcl, "--", "LineWidth", 2);
title("Tip Acceleration over Time");
ylabel("Tip Acceleration (mm/s/s)");
xlabel("Time (s)");
legend("X Position", "Y Position", "Z Position");
hold off;