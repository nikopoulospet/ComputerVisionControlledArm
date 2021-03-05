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

% send robot to home position
%home first to remove false ball reading
packet = pp.setSetpoints([0,0,0]);
pp.write(SERV_ID, packet);
pause(1);

% initialize and calibrate camera
CAM = Camera();
% turn on debug to see camera images
CAM.DEBUG = true;
% 4 seconds to put any objects in the area
disp("PUT AN OBJECT IN THE AREA");
pause;
%find an object and detect its center
[objects,colors] = CAM .detectImproved();
while size(objects) ~= 0
    for ball=1:1:1
        %convert the object position to task space position
        Object_location = [objects(ball,1),objects(ball,2)];
        % calculate initial position, final position, and grab position based on
        % object location from detect function
        pos = pp.getPosition();
        initialPosition = transpose(pp.fk3001(pos));
        % apply correction values to make picking more accurate
        topPosition = [Object_location(1)-20, Object_location(2), 70];
        grabPosition = [Object_location(1)-20, Object_location(2), 18];
        homePosition = [100, 0, 195];
        switch colors(ball)
            case "pink"
                sortPosition = [25 ,100 ,15];
            case "purple"
                sortPosition = [25 ,-100 ,15];
            case "yellow"
                sortPosition = [125 ,100 ,15];
            case "green"
                sortPosition = [125 ,-100 ,15];
        end
        raisePosition = [sortPosition(1), sortPosition(2), 70];
        points = [initialPosition; topPosition; grabPosition; topPosition; raisePosition; sortPosition; homePosition];
        gripper = [ 0 ; 0 ; 1 ; 1 ; 1 ; 0 ; 0];
         
        % show video feed
        CAM.cam.preview();
        k = 2;
        while k <= 7
            % loop though each points and generate quintic trajectory each time
            initialPosition = points(k-1,:);
            finalPosition = points(k,:);
            interpolator = Interpolator('quintic', 3);
            disp(finalPosition);
            % pause between motions so trajectories cannot merge
            
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
            pause(1);
            if gripper(k) == 1 && gripper(k-1) == 0
                pp.writeGripper(1962, 65);
            elseif gripper(k) == 0 && gripper(k-1) == 1
                pp.writeGripper(1962, 180);
            end
            k = k + 1;
            
        end
    end
    [objects,colors] = CAM.detectImproved();
end
pp.shutdown();