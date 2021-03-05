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

joint_Angles = [29.52,70.3,-36.61;
           22.8,17.74,-36.61;
           63.84,62.62,-17.41;
          29.52,70.3,-36.61];
      
points = [154.3290, 87.3863, 73.2396;
          115.3207,48.4764, 222.5871;
           70.2094, 142.9362, 70.0196;
           154.3290, 87.3863, 73.2396];
i = 0;
waitTime = 1;
x = 0;
loop = false;

num_points = 30;
while 1
    x = x + 1;
    i = i + 1;
    if loop
        disp(time)
        return;
    end
    if i == 5
        i = 1;
        loop = true;
    end
    current_pos = pp.getPosition();
    j1Traj = traj.cubic_traj(0,1,0,0,current_pos(1),joint_Angles(i,1));
    j2Traj = traj.cubic_traj(0,1,0,0,current_pos(2),joint_Angles(i,2));
    j3Traj = traj.cubic_traj(0,1,0,0,current_pos(3),joint_Angles(i,3));
    
    tic;
    start = toc;
    delta = toc - start;
    while delta < 1
        delta = toc - start;
        q1 = j1Traj(1) + j1Traj(2) *(delta) + j1Traj(3) * ((delta)^2) + j1Traj(4) * ((delta)^3);
        q2 = j2Traj(1) + j2Traj(2) *(delta) + j2Traj(3) * ((delta)^2) + j2Traj(4) * ((delta)^3);
        q3 = j3Traj(1) + j3Traj(2) *(delta) + j3Traj(3) * ((delta)^2) + j3Traj(4) * ((delta)^3);
        jointAngles = [q1,q2,q3];
        
        packet = pp.setSetpoints(jointAngles);
        pp.write(SERV_ID, packet);

    end    
        
    time(x) = toc;
    

end
