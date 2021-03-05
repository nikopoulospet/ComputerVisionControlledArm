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
SERV_ID = 1848;

startTimePlot = tic;
output_data = zeros(1,10);

% send robot to initial position
target_pos = [50;0;50];
figure(1);
% input 5 positions
for i = 1:1:5
    figure(1)
    % create subplots for x-z plane view of stick model setting coordinates
    subplot(1, 2, 1)
    joint_angles = pp.ik_3001_numerical(target_pos,model);
    disp("Target Position " + target_pos(1) + ", " + target_pos(2) + ", " + target_pos(3));
    end_pos = pp.fk3001(joint_angles);
    disp("End Position " + end_pos(1) + ", "+ end_pos(2) + ", " + end_pos(3) + " ");
    subplot(1, 2, 2)
    xlabel('X target')
    xlim([-50 200])
    ylabel('Z target')
    ylim([-50 200])
    % pass in new position using cursor on second subplot
    new_pos = ginput(1);
    target_pos = [new_pos(1); 0; new_pos(2)];
end


