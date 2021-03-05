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

%send robot to home position to ensure that it know where it is starting
%from
packet = pp.setSetpoints([0,0,0]);
pp.write(SERV_ID, packet);

pause(3);
% pass in a set interpoltaion time for the setSetpoints function
packet = pp.setSetpoints_Improved([0,0,-90],5000);
pp.write(SERV_ID, packet);
position = pp.getPosition();
% run loop until arm determinant approaches the singularity
while pp.eStop(position)
    disp(pp.jpDet(pp.jacob3001(position)));
    try
        position = pp.getPosition();
        model.drawStickModel(position);
        pause(0.1);
    catch
        pp.shutdown();
    end
end
pp.shutdown()