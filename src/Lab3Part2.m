%%
% Lab 3 part 2
% Takes encoder readings from getPositions function
% Puts the encoder readings into fk3001(jointAngles) using forward kinimatics
% Puts the end effector positions from fk3001 into ik3001 using inverse
% kinematics to get the joint angles back (validation of ik model)
% Graphs a stick model of the robot using the return values of ik
% Exports data to two csv files, one filled with joint angle data and the
% other with end effector position data and plots both over time
% 

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

runs = 0;


% run once to set up communication protocol
while(runs < 1)
    try
      % ID of server on the Nucleo
      SERV_ID = 1848;
      % ID of the read packet
      SERVER_ID_READ = 1910;
      % enables/disables debug prints
      DEBUG   = true;

      % Instantiate a packet
      packet = zeros(15, 1, 'single');

      % get the packet from the Nucleo
      test = pp.read(SERVER_ID_READ);

      % read the angles from the packet
      joint_angles = pp.getPosition();
    
      % takes the joint angels and returns the end effector position
      end_effector_position = pp.fk3001(joint_angles);

      % takes the joint angles and plots the stick model of the arms motion 
      model.drawStickModel(joint_angles);


      if DEBUG
          disp('Joint angles: ');
          disp(joint_angles);
          disp('End effector position: ');
          disp(end_effector_position);
      end

      catch exception
        getReport(exception)
        disp('Exited on error, clean shutdown');
        pp.shutdown()
    end
    
    
    pause(0.25);
    runs = runs + 1;
end

% send to home position initially and pause to ensure robot finishes motion
packet = pp.setSetpoints([0,0,0]);
pp.write(SERV_ID, packet);
pause(3);

% send to triangle positions individually
tic;
viaPts = [-29.52,70.3,-36.61]; %Position 1
%viaPts = [22.8,17.74,-36.61]; %Position 2
%viaPts = [63.84,62.62,-17.41]; %Position 3

% create arbitrary wait time of 1.5 seconds
waitTime = 1.5;
packet = pp.setSetpoints(viaPts);
pp.write(SERV_ID, packet);
    
% creates a time-based while loop using an offset time  
count = toc;
offsetTime = toc;

    while(count < waitTime + offsetTime)
      test = pp.read(SERVER_ID_READ);
      
      % read the angles from the packet, convert to end effector positions,
      % convert back to joint angles to validate inverse kinematics
      initial_angles = pp.getPosition();
      positions = pp.fk3001(initial_angles);
      joint_angles = pp.ik3001(positions);
     
      % takes the joint angles from ik and plots the stick model of the arms motion 
      figure(1)
      model.drawStickModel(joint_angles);
      pause(0.0001);
      
      % send joint angle and end effector positions to separate csv files
      motorpos = [joint_angles(1) joint_angles(2) joint_angles(3) toc];
      writematrix(motorpos, 'MotorPositionsLab3.csv', 'WriteMode', 'append');
      tipmatrix = [positions(1) positions(2) positions(3)];
      writematrix(tipmatrix, 'TipPositionsLab3.csv', "WriteMode", "append");
      
      count = toc;
    end

% load csv files and extract data by column
load MotorPositionsLab3.csv;
motor1 = MotorPositionsLab3(:,1);
motor2 = MotorPositionsLab3(:,2);
motor3 = MotorPositionsLab3(:,3);
time = MotorPositionsLab3(:,4);

load TipPositionsLab3.csv;
xPos = TipPositionsLab3(:,1);
yPos = TipPositionsLab3(:,2);
zPos = TipPositionsLab3(:,3);

% plots Joint Angles over Time
figure(2)
grid on;
hold on;
plot(time, motor1, "--", "LineWidth", 2);
plot(time, motor2, "-o", "LineWidth",2);
plot(time, motor3, "-.", "LineWidth", 2);
title("Joint Angles over Time");
ylabel("Joint Angles (deg)");
xlabel("Time (s)"); 
legend("Joint 1 Position", "Joint 2 Position", "Joint 3 Position");
hold off;

% plots end effector positions in x,y,z axes
figure(3)
hold on;
grid on;
plot(time, xPos, "-o", "LineWidth", 2);
plot(time, yPos, "-.", "LineWidth", 2);
plot(time, zPos, "--", "LineWidth", 2);
title("Tip Positions over Time");
ylabel("Tip Position (mm)");
xlabel("Time (s)"); 
legend("X Position", "Y Position", "Z Position");
hold off;
pp.shutdown()


