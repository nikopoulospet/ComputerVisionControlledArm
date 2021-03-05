classdef model
    properties
        xLimits = [-100,300];
        yLimits = [-200,200];
        zLimits = [-100,300];
    end
    methods
        function drawStickModel(model,jointAngles)
            %DrawStickModel function takes in a 3x1 array representing q
            %q1, q2, q3 represent joint angles of the robot arm
            %this function is meant to be called in a loop to constanly update the
            %image.
            
            %apply DH paramters
            T = Robot.applyDHparameters(jointAngles * (pi/180));
            T01 = T(:,:,1);
            T02 = T01 * T(:,:,2);
            T03 = T02 * T(:,:,3);
            T04 = T03 * T(:,:,4);
            
            %create matrix of joint positions x1 = frame 1, y1 = x coord
            %y2 = y coord ect...
            joints = [0,0,0; ...
                      transpose(T01(1:3,4)); ...
                      transpose(T02(1:3,4)); ...
                      transpose(T03(1:3,4)); ...
                      transpose(T04(1:3,4))];
            
            %plot with some formatting
            plot3(joints(:,1),joints(:,2),joints(:,3),'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);grid on;
            xlim(model.xLimits)
            ylim(model.yLimits)
            zlim(model.zLimits)
        end
        
        function drawWorkspace(model,JointAngleRanges)
            %will plot all end effector positons to a 3D graph
            %joint angle Ranges is a 3x2 matrix which includes the start and end ranges of each joint.
            resolution = 5;
            count = 0;
            
            %JointAngleRanges = [-150,140;-50,96;-85,68];
            
            for joint1 = JointAngleRanges(1,1):resolution:JointAngleRanges(1,2)
                for joint2 = JointAngleRanges(2,1):resolution:JointAngleRanges(2,2)
                    for joint3 = JointAngleRanges(3,1):resolution:JointAngleRanges(3,2)
                        count = count + 1;
                        temp = Robot.fk3001([joint1,joint2,joint3]);
                        x(count) = temp(1);
                        y(count) = temp(2);
                        z(count) = temp(3);
                    end
                end
            end
            
            plot3(x,y,z,'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF')
        end
            
            
    end
    
end