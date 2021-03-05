close all
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
% initialize and calibrate camera
CAM = Camera();
% turn on debug to see camera images
CAM.DEBUG = true;
% 4 seconds to put any objects in the area
disp("PUT AN OBJECT IN THE AREA");
pause(4);
%find an object and detect its center
object_center = CAM.detect();

%convert the object position to task space position
Object_location = CAM.calcPositions([object_center(1,1),object_center(1,2)]);

% send robot to home position
packet = pp.setSetpoints([0,0,0]);
pp.write(SERV_ID, packet);
% open servo and wait 2 seconds
pp.writeGripper(1962, 180);
pause(2);

% calculate initial position, final position, and grab position based on
% object location from detect function
pos = pp.getPosition();
initialPosition = transpose(pp.fk3001(pos))
% apply correction values to make picking more accurate
finalPosition = [Object_location(1)-25, Object_location(2)-2, 50];
grabPosition = [Object_location(1)-25, Object_location(2)-2, 18];
points = [initialPosition; finalPosition; grabPosition];

% show video feed
CAM.cam.preview();
k = 2;
while k <= 3
  % loop though each points and generate quintic trajectory each time
  initialPosition = points(k-1,:);
  finalPosition = points(k,:);
  interpolator = Interpolator('quintic', 5);
  disp(finalPosition);
  % pause between motions so trajectories cannot merge
  pause(2);
  tic;
  while 1
    % since tic starts between each position, deltaTime is toc since this
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
    packet = pp.setSetpoints_Improved(motorAngles, 1000);
    pp.write(SERV_ID, packet);
    pause(.01);

    % to break infinite while loop, scalar approximately 1 so it completes
    % the path
    if scaler >= .99
        scaler = 0;
        break;
    end 
   end
 k = k + 1;     
end
pause(2);
% close gripper and wait before shutdown
pp.writeGripper(1962, 65);
pause(1);
pp.shutdown();