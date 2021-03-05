
classdef Robot
    properties
        %hidDevice;
        %hidService;
        myHIDSimplePacketComs
        pol
    end
    properties(Constant)
        DHparamters = [0,55,0,0;
            0,40,0,-(pi/2);
            -(pi/2) ,0,100, 0;
            (pi/2) ,0,100, 0];
        %add joint angles to DH pareamters when used
        
    end
    methods
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(packet)
            %Close the device
            packet.myHIDSimplePacketComs.disconnect();
        end
        % Create a packet processor for an HID device with USB PID 0x007
        function packet = Robot(dev)
            packet.myHIDSimplePacketComs=dev
            packet.pol = java.lang.Boolean(false);
        end
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
            com= zeros(15, 1, 'single');
            try
                ds = javaArray('java.lang.Double',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                for i=1:length(com)
                    com(i)= ret(i).floatValue();
                end
                %class(com)
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        function com = read(packet, idOfCommand)
            com= zeros(15, 1, 'single');
            try
                
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                for i=1:length(com)
                    com(i)= ret(i).floatValue();
                end
                %class(com)
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        function write(packet, idOfCommand, values)
            try
                ds = javaArray('java.lang.Double',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Double(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);
                
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        function positions = getPosition(packet)
            data = zeros(15, 1, 'single');
            VELOCITY_SERVER_ID = 1910;
            positions = [0;0;0];
            try
                % Default packet size for HID
                intid = java.lang.Integer(VELOCITY_SERVER_ID);
                
                ret = packet.myHIDSimplePacketComs.readFloats(intid) ;
                for j = 1 : length(data)
                    data(j) = ret(j).floatValue();
                end
                
                %class(com)
                positions(1) = data(3);
                positions(2) = data(5);
                positions(3) = data(7);
                
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        function velocities = getVelocities(packet)
            vel = zeros(15, 1, 'single');
            VELOCITY_SERVER_ID =1822;
            velocities = [0;0;0];
            try
                
                % Default packet size for HID
                intid = java.lang.Integer(VELOCITY_SERVER_ID);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                for i=1:length(vel)
                    vel(i)= ret(i).floatValue();
                end
                %class(com)
                velocities(1) = vel(3);
                velocities(2) = vel(6);
                velocities(3) = vel(9);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % setSetpoints takes in a 1x3 position matrix and creates a send packet
        % using the input position values
        function posPacket = setSetpoints(packet, positionMatrix)
            [m,n] = size(positionMatrix);
            if(m == 1 && n == 3)
                posPacket = zeros(15, 1, 'single');
                posPacket(1) = 1000;%one second time
                posPacket(2) = 0;%linear interpolation
                posPacket(3) = positionMatrix(1);
                posPacket(4) = positionMatrix(2);% Second link to 0
                posPacket(5) = positionMatrix(3);% Third link to 0
                
            else
                disp("Please input a 1x3 matrix");
            end
        end
      %  function posPacket = setGripper(packet, gripperPos)
       %         posPacket = zeros(15, 1, 'single');
       %         posPacket(1) = gripperPos;%one second time
       % end
        
        function writeGripper(packet, idOfCommand, values)
            try
                ds = javaArray('java.lang.Byte',length(values));
                for i=1:length(values)
                    ds(i)= java.lang.Byte(values(i));
                end
                % Default packet size for HID
                intid = java.lang.Integer(idOfCommand);
                %class(intid);
                %class(idOfCommand);
                %class(ds);
                %write bytes for servo command instead of floats like the
                %motors
                packet.myHIDSimplePacketComs.writeBytes(intid,ds,packet.pol);
                
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        function posPacket = setSetpoints_Improved(packet, positionMatrix,time)
            [m,n] = size(positionMatrix);
            if(m == 1 && n == 3)
                posPacket = zeros(15, 1, 'single');
                posPacket(1) = time;%one second time
                posPacket(2) = 0;%linear interpolation
                posPacket(3) = positionMatrix(1);
                posPacket(4) = positionMatrix(2);% Second link to 0
                posPacket(5) = positionMatrix(3);% Third link to 0
                
            else
                disp("Please input a 1x3 matrix");
            end
        end
        function jointAngles = ik3001(packet, endEffectorPosition)
            % calculates the inverse kinematics of the robot arm given end
            % effector positions, assume elbow up configuration and only
            % positive angles
            % create values for given positions of end effector
            xc = endEffectorPosition(1);
            yc = endEffectorPosition(2);
            zc = endEffectorPosition(3);
            
            % set up limits for robot, found from calculating distance
            % formula and moving arm to max positions on the board
            % if distance formula result is less than 53mm, then robot would
            % collide with the base (calculating radius from 0,0,0)
            centerRadius = 53; % center radius is 53mm
            xMin = -40;
            xMax = 180;
            yMin = -133;
            yMax = 180;
            zMin = -40;
            zMax = 293;
            
            
            distanceFromZero = sqrt(xc^2 + yc^2 + zc^2);
            
            if((distanceFromZero >= centerRadius) && (yMin <= yc) && (yc <= yMax) && (xMin <= xc) && (xc <= xMax) && (zMin <= zc) && (zc <= zMax))
                % define robot geometry parameters
                r = sqrt(xc^2 + yc^2);
                d1 = 95; % length of link 1
                s = zc - d1;
                h = sqrt(r^2 + s^2);
                L2 = 100; % length of link 2
                L3 = 100; % length of link 3 in mm
                
                theta1 = atan2(yc,xc);
                
                % define parameters and solve for Theta3
                D3 = -((L2^2 + L3^2 - h^2)/(2 * L2 * L3)); % define cos(theta3)
                C3 = sqrt(1-D3^2);
                theta3Prime = atan2(C3,D3);
                theta3 = (pi/2) - theta3Prime;
                
                
                alpha = atan2(s,r);
                
                % calculate theta2' using geometric approach then 90 - theta2'
                % to get theta2 with respect to home position of robot
                DBeta = (L2^2 + h^2 - L3^2)/(2 * L2 * h);
                CBeta = sqrt(1-DBeta^2);
                beta = atan2(CBeta,DBeta);
                theta2Prime = alpha + beta;
                theta2 = (pi/2) - theta2Prime;
                
                theta1 = theta1 * (180/pi);
                theta2 = theta2 * (180/pi);
                % theta3 made negative since motor rotates the opposite
                % direction
                theta3 = theta3 * (180/pi) * -1;
                jointAngles = [theta1,theta2,theta3];
                
                % send error if position is out of range of the robot and shut down
                % SimplePacketComms communication
            else
                msg = "Error! Position is out of range!"
                packet.shutdown();
                error(msg);
            end
        end
       function jacobian = jacob3001(packet, q)
            %create all transformation matrices
            jointAngles = q * (pi/180);
            %starting with T01 fixes problems with jacobian
            T01 = Robot.transformation(0, 55, 0, 0);
            T12 = Robot.transformation(jointAngles(1), 40, 0, -(pi/2));
            T23 = Robot.transformation((jointAngles(2) - (pi/2)), 0, 100, 0);
            T34 = Robot.transformation((jointAngles(3) + (pi/2)), 0, 100, 0);
            % used for calculating J-orientation and z unit vectors
            T04 = T01 * T12 * T23 * T34; 
            T02 = T01 * T12;
            T03 = T01 * T12 * T23;
            Pe = T04(1:3,4); % end effector position
            %calculate z unit vectors
            z01 = T01(1:3,3);
            z02 = T02(1:3,3);
            z03 = T03(1:3,3);
            unitz01 = z01/(sqrt(z02(1)^2 + z02(2)^2 + z02(3)^2));
            unitz02 = z02/(sqrt(z03(1)^2 + z03(2)^2 + z03(3)^2));
            unitz03 = z03/(sqrt(z03(1)^2 + z03(2)^2 + z03(3)^2));
            %calculate cross products
            Jp1 = cross(unitz01, (Pe - T01(1:3,4)));
            Jp2 = cross(unitz02, (Pe - T02(1:3,4)));
            Jp3 = cross(unitz03, (Pe - T03(1:3,4)));
            Jp = [Jp1, Jp2, Jp3];
            Jo = [T02(1:3,3),T03(1:3,3),T04(1:3,3)];
            jacobian = [Jp;Jo];
       end
       function pdot = fdk3001(packet, q, qdot)
            % q is the vector of current joint vars
            % qdot is the vector of current joint velocities
            %solves pdot = j(q) * qdot
            jacob = packet.jacob3001(q);
            jacob2 = jacob(1:3,:);
            pdot = jacob2 * qdot;
       end
       function ikSol = ik_3001_numerical(packet, target_pos, model)
           % set arbitrary initial joint angles and error
           position = packet.getPosition();
           qi = position;
           e = 5;
           error = target_pos - packet.fk3001(qi);
           %interpolate towards the target
           while sum(abs(error)) > e
               %define mag of end effector vel for each step
               error = target_pos - packet.fk3001(qi);
               jacob = packet.jacob3001(qi);
               deltaQ = pinv(jacob(1:3,:)) * (error);
               %use inverse jacob to convert ee vel to j vel
               qi = qi + deltaQ;
               %model.drawStickModel(qi);
               % set the view to the x-z plane
               %view(0,0);
               %pause(0.01);
               %multiply joint vel with time int to get inc amount
               %inc until target is reached
           end
           ikSol = qi;
       end
       
       function safe = eStop(packet, pos)
            determanant = packet.jpDet(packet.jacob3001(pos));
            if abs(determanant) < 1000
                figure(1)
                text(100,100,100,"E-STOP");
                disp("Aproching Sigularity E-STOPING")
                packet.shutdown()
                safe = 0;
            else
                safe = 1;
            end
        end
    end
    
    
    
    methods(Static)
        function tMatrix = transformation(theta, d, a, alpha)
            % creates transformation matrix given D-H parameters
            tMatrix = [cos(theta) -(sin(theta) * cos(alpha)) sin(theta) * sin(alpha) a * cos(theta);
                sin(theta) cos(theta) * cos(alpha) -(cos(theta) * sin(alpha)) a * sin(theta);
                0 sin(alpha) cos(alpha) d;
                0 0 0 1;];
        end
        
        function arrayofTmatrix = applyDHparameters(jointAngles)
            %iterate though joint angles applying DH paramters to each frame
            %array of T matrix is a 4X4X4 matrix. address each frame as
            %follows: ex (:,:,1) == frame0 to 1
            for x = 1:1:4
                if x > 1
                    arrayofTmatrix(:,:,x) = Robot.transformation(jointAngles(x-1) + Robot.DHparamters(x,1), ...
                        Robot.DHparamters(x,2),Robot.DHparamters(x,3),Robot.DHparamters(x,4));
                else
                    arrayofTmatrix(:,:,x) = Robot.transformation(Robot.DHparamters(x,1), ...
                        Robot.DHparamters(x,2),Robot.DHparamters(x,3),Robot.DHparamters(x,4));
                end
            end
            
        end
        
        function endEffectorPosition = fk3001(jointAngles)
            % Calculates position of end effectors based on input joint angles
            % through the use of forward kinematics transformation matrices and D-H
            % parameters
            
            %generate transformation matrices based on D-H variables found by following
            %the D-H convention of the Hephaestus Arm
            
            
            % Since transformation uses radians, this value must be converted to radians
            jointAngles = jointAngles * (pi/180);
            
            %apply the DH paramters
            T = Robot.applyDHparameters(jointAngles);
            
            % apply post multiplication
            T04 = T(:,:,1) * T(:,:,2) * T(:,:,3) * T(:,:,4);
            
            % extract position matrix from final transformation matrix
            endEffectorPosition = T04(1:3,4);
            
        end
        function determinantJp = jpDet(jacobian)
            %calculate determinant of position jacobian to check if a
            %singularity exists
            Jp = jacobian(1:3,:);
            determinantJp = det(Jp);

      end
    end
end
