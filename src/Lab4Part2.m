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

% joint_Angles = [29.52,70.3,-36.61;
%            22.8,17.74,-36.61;
%            63.84,62.62,-17.41;
%           29.52,70.3,-36.61];

%setpoints to create a star in a 3D-ish plane
point1 = [154.3290, 87.3863, 73.2396];
point2 = [115.3207,48.4764, 222.5871];
point3 = [70.2094, 142.9362, 70.0196];
points = [[100,0,195];point1;point2;point3;point1];

% start k=2 and end at an index higher so we can access elements in points
k = 2;
% start timer for plotting, so it doesnt reset
startTimePlot = tic;
output_data = zeros(1,10);
%send robot to home position to ensure that it know where it is starting
%from
% first position of points matrix and the robot starting position should
% match
packet = pp.setSetpoints([0,0,0]);
pp.write(SERV_ID, packet);

while k <= 5
  % grab first and last positions of each setpoint pair
  initialPosition = points(k-1,:);
  finalPosition = points(k,:);
  
  % set end time to 5 seconds, 2 seconds was too little
  interpolator = Interpolator('quintic', 5);
  tic;
  
  while 1
    % since tic starts betw scaler = 0;een each position, deltaTime is toc since this
    % is the time elapsed since the movement started
    deltaTime = toc;
    scaler = interpolator.getScaler(deltaTime);
    
    xMin = initialPosition(1);
    xMax = finalPosition(1);
    yMin = initialPosition(2);
    yMax = finalPosition(2);
    zMin = initialPosition(3);
    zMax = finalPosition(3);
    
    % calculate positions using scaler
    x = (xMax - xMin) * scaler + xMin;
    y = (yMax - yMin) * scaler + yMin;
    z = (zMax - zMin) * scaler + zMin;
    positions = [x, y, z];
    motorAngles = pp.ik3001(positions);
    % send packet and pause for .01s between each loop so robot can update
    packet = pp.setSetpoints(motorAngles);
    pp.write(SERV_ID, packet);
    pause(.01);
    
    position = pp.getPosition();
    velocities = pp.getVelocities();
    accelerations = velocities/deltaTime;
    
    % draw updating stick model, hold on has to go after the first is drawn
    figure(1)
    
    model.drawStickModel(position);
    % create linear velocity vector and plot it using the quiver 3 function
    pdot = pp.fdk3001(position, velocities);
    endEffector_pos = pp.fk3001(position);
    hold on
    quiver3(endEffector_pos(1),endEffector_pos(2),endEffector_pos(3),pdot(1),pdot(2),pdot(3));
    hold off
    %hold on;
    currentTime = toc(startTimePlot);
    
   
    
    
    curr_data_matrix = [positions(1) positions(2) positions(3) velocities(1) velocities(2) velocities(3) ...
                   accelerations(1) accelerations(2) accelerations(3) currentTime];
    
    %concatenate data matrix
    output_data = cat(1,output_data,curr_data_matrix);
    
    
    % to break infinite while loop, scalar approximately 1 so it completes
    % the path
    if scaler >= .99
        scaler = 0;
        break;
    end    
  end
k = k + 1;   
end
pp.shutdown()