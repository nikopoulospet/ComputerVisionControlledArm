% part 5.2 lab 3
% recording the planned reajectory
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
model = model();
trajPlanner = Traj_planner();
SERV_ID = 1848;
vertex1 = [-29.52,70.3,-36.61]; %Position 1
vertex2 = [22.8,17.74,-36.61]; %Position 2
vertex3 = [63.84,62.62,-17.41]; %Position 3

viaPts = [pp.fk3001(vertex1),pp.fk3001(vertex2),pp.fk3001(vertex3),pp.fk3001(vertex1)];

%send the robot to the first vertex
packet = pp.setSetpoints(vertex1);
pp.write(SERV_ID, packet);
pause(1);
tic;
cur_time = toc;



packet = zeros(15, 1, 'single');

if(1)
    tic;
    for point = 1:1:length(viaPts)-1
        %Get the two points to go between
        first_pt = [viaPts(1,point),viaPts(2,point),viaPts(3,point)];
        second_pt = [viaPts(1,point+1),viaPts(2,point+1),viaPts(3,point+1)];
        
        %get the coefficants of the linear trajectory
        cur_edge_coeffs = trajPlanner.linear_traj(first_pt,second_pt,0,.5);
        
        
        for coeff = 1:1:length(cur_edge_coeffs)-1
            last_time = toc;
            while (last_time + .5 > toc)
                %get the current time
                cur_time = toc - last_time;
                
                %plug the current time into the task space trajectory equation to
                %get the target position
                x_pos = cur_edge_coeffs(coeff,1) + cur_edge_coeffs(coeff,2) * cur_time;
                y_pos = cur_edge_coeffs(coeff,3) + cur_edge_coeffs(coeff,4) * cur_time;
                z_pos = cur_edge_coeffs(coeff,5) + cur_edge_coeffs(coeff,6) * cur_time;
                end_effector_pos = double([x_pos,y_pos,z_pos]);
                
                %convert the end effector position to joint space position
                joint_angels = pp.ik3001(end_effector_pos);
                
                %send the joint space positions to the robot
                packet = pp.setSetpoints(joint_angels);
                pp.write(SERV_ID, packet);
                
                positions = pp.getPosition();
                task_space_pos = pp.fk3001(positions);
                pos_end = [task_space_pos(1),task_space_pos(2),task_space_pos(3),toc];
                writematrix(pos_end, 'Part5_2_b.csv', "WriteMode", "append");
            end
        end
    end
end
pp.shutdown();

%make the 3d graph
load Part5_2_a.csv;
xPos = Part5_2_a(:,1);
yPos = Part5_2_a(:,2);
zPos = Part5_2_a(:,3);

figure(1)
hold on;
grid on;
plot3(xPos,yPos,zPos,'r');
title("Tip Positions in task spave");
ylabel("Position Y (mm)");
xlabel("Position X (mm)");
zlabel("Position Z (mm)");
hold off;

%make the pos graph
load Part5_2_b.csv;
xPos = Part5_2_b(:,1);
yPos = Part5_2_b(:,2);
zPos = Part5_2_b(:,3);
time = Part5_2_b(:,4);

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
load Part5_2_b.csv;
xPos = Part5_2_b(:,1);
yPos = Part5_2_b(:,2);
zPos = Part5_2_b(:,3);
time = Part5_2_b(:,4);

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
load Part5_2_b.csv;
xPos = Part5_2_b(:,1);
yPos = Part5_2_b(:,2);
zPos = Part5_2_b(:,3);
time = Part5_2_b(:,4);

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