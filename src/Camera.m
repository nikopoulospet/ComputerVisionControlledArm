classdef Camera
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = false;
        DEBUG_BALLDETECTION = false;
        
        % Image Processing Variables
        
        % Colors
        
        % Properties
        params;
        TimgChecker;
        cam;
        cam_pose;
    end
    
    methods
        
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            self.TimgChecker = self.getCameraPose(); % calculate camera pose
        end
        
        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
        
        function [pose, color] = detectBestBall(self)
            % DETECTBESTBALL Detect Balls + Report back positions + colors
            % Note: you do NOT need to use this function, this is just a recommendation
            
        end
        
        function params = calibrate(self)
            % CALOBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful
            
            % NOTE: This uses the camcRBE 3001 -Lab 5 Project105.Object detection and classification5.1.Object detection and localizationUsing the image processing methods covered in class, implement a function that takes as input a frame from the camera and identifies the centroid of a solid colored spherical tracking object (i.e. the color balls that came with your robot kit) and overlay a marker on the displayed video feed. As a first operation, you will need toun-distort the image captures from the camera using the matrix of camera parameters estimated earlier. Adalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below
            
            DEBUG = self.DEBUG;
            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                camcalib; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                getReport(exception);
                disp("No camerea calibration file found. Plese run camera calibration");
            end
        end
        
        function pose = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!
            
            % There are a few debugging options included as well! Simply set POSE_PLOT
            % to true to show the checkerboard frame of reference on the picture!
            
            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, new_origin] = undistortImage(raw_img, self.params, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Adjust imagePoints so they are in the same frame of
            % reference as original image
            imagePoints = imagePoints + new_origin;
            % 5. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, self.params);
            
            pose = [   R,    t';
                0, 0, 0, 1];
            
            if self.POSE_PLOT
                axesPoints = worldToImage(self.params, R, t, [0 0 0; 0 50 0; 50 0 0]);
                
                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';
                
                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';
                
                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
                
                imshow(img)
                title('Undistorted Image with checkerboard axes');
                
                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');
                
            end
        end
        
        function [imgPoints colors] = detect(self)
            close all
            checker_size = 25;
            
            % Capture image from camera
            raw_img =  snapshot(self.cam);
            
            % Undistort Image based on params
            [img, new_origin] = undistortImage(raw_img, self.params, 'OutputView', 'full');
            
            % Convert the image to the HSV color space.
            imHSV = rgb2hsv(img);
            
            % Get the saturation channel.
            saturation = imHSV(:, :, 2);
            
            % Threshold the image
            t = graythresh(saturation);
            
            
            imgBall = (saturation > t);
            se = strel('sphere',5);
            imgBall = imerode(imgBall, se);
            
            if (self.DEBUG)
                figure; imshow(imgBall, 'InitialMagnification', checker_size);
                title('Segmented Balls');
            end
            
            blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,'CentroidOutputPort', false,'BoundingBoxOutputPort', true,'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);
            [areas, boxes] = step(blobAnalysis, imgBall);
            
            % Sort connected components in descending order by area
            [~, idx] = sort(areas, 'Descend');
            
            % Get the 'numBalls' largest components.
            %boxes = double(boxes(idx(1:numBalls), :));
            
            if (self.DEBUG)
                imDetectedCoins = insertObjectAnnotation(img, 'rectangle', ...
                    1 * boxes, 'ball');
                figure(5);
                imshow(imDetectedCoins);
                title('Detected Balls');
            end
            balls_found = 0;
            % return the center of the ball
            for item = 1:1:size(boxes,1)
                x_pixel = boxes(item,1) + boxes(item,3)/2;
                y_pixel = boxes(item,2) + boxes(item,4)/2;
                pos = self.calcPositions([x_pixel, y_pixel]);
                color = self.getImgColor(boxes(item,:),imHSV);
                disp(color);
                if (abs(pos(2)) < 125 && pos(1) < 175 && pos(1) > 0)
                    balls_found = balls_found + 1;
                    imgPoints(balls_found, 1) = x_pixel;
                    imgPoints(balls_found, 2) = y_pixel;
                    colors(balls_found,1) = color;
                    if (self.DEBUG)
                        figure(5);
                        hold on
                        plot (imgPoints(balls_found, 1), imgPoints(balls_found,2), 'o', 'MarkerSize', 4, 'LineWidth', 2);
                    end
                end
            end
            
        end
        function colorSTR = getImgColor(self, box, HSVimg)
            stdDev = 0.04;
            yellow = 0.14; % 0.03
            green = 0.27; % 0.03
            pink = 0.90; % 0.03
            purple = 0.67; % 0.02
            
            hue = 0;
            count = 0;
            for j = box(1):1:box(1)+box(3)
                for i = box(2):1:box(2)+box(4)
                    %if this is less than some vibrancy then ignore
                    if HSVimg(i,j,3) > 0.25 && HSVimg(i,j,2) > .25
                        count = count + 1;
                        hue = hue + HSVimg(i,j,1);
                    end
                end
            end
            color = hue /count;
            if(self.DEBUG)
                disp("HUE: " + color)
            end
            if color >= purple - stdDev && color <= purple + stdDev
                colorSTR = "purple";
                return
            end
            if color >= yellow - stdDev && color <= yellow + stdDev
                colorSTR = "yellow";
                return
            end
            if color >= green - stdDev && color <= green + stdDev
                colorSTR = "green";
                return
            end
            if color >= pink - stdDev && color <= pink + stdDev
                colorSTR = "pink";
                return
            end
            colorSTR = "NONE";
            return
        end
        function Po = calcPositions(self, pixelPos)
            % camera pose returns the transformation from image to checkerboard
            RimgChecker = self.TimgChecker(1:3, 1:3);
            pimgChecker = self.TimgChecker(1:3, 4);
            
            % calculated by hand
            TbaseChecker = [0 1 0 75; 1 0 0 -100; 0 0 -1 0; 0 0 0 1;];
            
            % position in the checker frame, takes care of TimgChecker so need for it
            % in multiplication to find Po
            Pchecker =  pointsToWorld(self.params, RimgChecker, pimgChecker, pixelPos);
            Pchecker = transpose(Pchecker);
            
            ballWidth = 11; %distance to center of ball in mm
            Pi = [Pchecker; ballWidth; 1;];
            
            Po = TbaseChecker * Pi;
        end
        function [unsortedBalls unsortedColors] = detectImproved(self)
            [positions, colors] = self.detect();
            sortRadius = 45;
            unsortedBalls = [];
            unsortedColors = [];
            for ball=1:1:size(positions,1)
                pos = transpose(self.calcPositions(positions(ball,:)));
                switch colors(ball)
                    case "pink"
                        sortPosition = [25 ,100 ,18];
                    case "purple"
                        sortPosition = [25 ,-100 ,18];
                    case "yellow"
                        sortPosition = [125 ,100 ,18];
                    case "green"
                        sortPosition = [125 ,-100 ,18];
                    case "NONE"
                        sortPosition = pos;
                end
                xDifference = pos(1) - sortPosition(1);
                yDifference = pos(2) - sortPosition(2);
                xDifference = cast(xDifference, 'double');
                yDifference = cast(yDifference, 'double');
                distanceFromCentroid = sqrt(xDifference^2 + yDifference^2);
                if(distanceFromCentroid >= sortRadius)
                    unsortedBalls = [unsortedBalls; pos(1:2)];
                    unsortedColors = [unsortedColors; colors(ball)];
                end
            end
            
        end
    end
end
