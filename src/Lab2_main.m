%
% Lab 2 main 
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


%runs the while loop 100 time
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

% random setpoints for the robot to move too
viaPts = [[50,76,-56];[18,-60,68];[-54,20,77];[-75,74,50];[33,-51,3]];

%time to wait while the robot moves to the setpoint
waitTime = 3;

for i = 1:1:5
    
    %sets the setpoint for the robot to move to
    packet = pp.setSetpoints([viaPts(i , 1),viaPts(i , 2),viaPts(i , 3)]);
    pp.write(SERV_ID, packet);
    
    tic; %start timer
    while(toc < waitTime)
      test = pp.read(SERVER_ID_READ);

      % read the angles from the packet
      joint_angles = pp.getPosition();

      % takes the joint angles and plots the stick model of the arms motion 
      model.drawStickModel(joint_angles);
      pause(0.01);
    end
end

pp.shutdown()

