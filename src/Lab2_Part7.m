%%
% Lab 2 part 7
% Takes encoder reading
% Puts the encoder readings into fk3001(jointAngles) forward kinimatics
% Takes the encoder readings and puts them into the stick model function
% TODO: make it contionous

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

% positions of arbitrary triangle created by moving robot to three
% positions keeping the first joint at 0 degrees
viaPts = [[0,36.14,33.5];[0,38.38,-12.13];[0,0.5,24.6];[0,36.14,33.5]];
waitTime = 1;
tic;

%loops for each setpoint
for i = 1:1:4
    %extracts joint angles for each setpoint and sends a packet
    packet = pp.setSetpoints([viaPts(i , 1),viaPts(i , 2),viaPts(i , 3)]);
    pp.write(SERV_ID, packet);
    
    % creates a time-based for loop using an offset time
    count = toc;
    offsetTime = toc;
    while(count < waitTime + offsetTime)
      test = pp.read(SERVER_ID_READ);

      % read the angles from the packet
      joint_angles = pp.getPosition();
      
      % takes the joint angles and plots the stick model of the arms motion 
      figure(1)
      model.drawStickModel(joint_angles);
      pause(0.01);
      
      % exports joint angles and end effector positions to csv files
      motorpos = [joint_angles(1) joint_angles(2) joint_angles(3) toc];
      writematrix(motorpos, 'MotorPositions.csv', 'WriteMode', 'append');
      tipPosition = pp.fk3001([joint_angles(1),joint_angles(2),joint_angles(3)]);
      tipmatrix = [tipPosition(1) tipPosition(2) tipPosition(3)];
      writematrix(tipmatrix, 'TipPositions.csv', "WriteMode", "append");
      
      count = toc;
    end
end


% loads csv files and extracts data into columns
load MotorPositions.csv;
motor1 = MotorPositions(:,1);
motor2 = MotorPositions(:,2);
motor3 = MotorPositions(:,3);
time = MotorPositions(:,4);

load TipPositions.csv;
xPos = TipPositions(:,1);
yPos = TipPositions(:,2);
zPos = TipPositions(:,3);


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

% plots end effector positions in x and z axes
figure(3)
hold on;
grid on;
plot(time, xPos, "-o", "LineWidth", 2);
plot(time, zPos, "-.", "LineWidth", 2);
title("Tip Positions in X and Z Axes");
ylabel("Tip Position (mm)");
xlabel("Time (s)"); 
legend("X Position", "Z Position");
hold off;

% finds positions of setpoints for graphing
posSetpoint1 = pp.fk3001([0,36.14,33.5]);
posSetpoint2 = pp.fk3001([0,38.38,-12.13]);
posSetpoint3 = pp.fk3001([0,0.5,24.6]);

% plots 2D path in x-z plane 
figure(4)
hold on;
grid on;
plot(xPos, zPos, "-o", "LineWidth", 2);
plot(posSetpoint1(1), posSetpoint1(3), "r*");
plot(posSetpoint2(1), posSetpoint2(3), "g*");
plot(posSetpoint3(1), posSetpoint3(3), "m*");
title("Tip Positions in X-Z Plane");
ylabel("Z Position (mm)");
xlabel("X Position (mm)"); 
legend("Position in X-Z Plane", "Setpoint 1", "Setpoint 2", "Setpoint 3");
hold off;

% plots 2D path in x-y plane 
figure(5)
hold on;
grid on;
plot(xPos, yPos, "-o", "LineWidth", 2);
plot(posSetpoint1(1), posSetpoint1(2), "r*");
plot(posSetpoint2(1), posSetpoint2(2), "g*");
plot(posSetpoint3(1), posSetpoint3(2), "m*");
title("Tip Positions in X-Y Plane");
ylabel("Y Position (mm)");
xlabel("X Position (mm)"); 
legend("Position in X-Y Plane", "Setpoint 1", "Setpoint 2", "Setpoint 3");
hold off;

pp.shutdown()

