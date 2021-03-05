clear;
cam = Camera();
Po1 = calcPositions(cam, [201 205]);
Po2 = calcPositions(cam, [102 331]);
Po3 = calcPositions(cam, [524 368]);
Po4 = calcPositions(cam, [510 211]);


disp("First Point: X: " + Po1(1) + " Y: " + Po1(2) + " Z: " + Po1(3));
disp("Second Point: X: " + Po2(1) + " Y: " + Po2(2) + " Z: " + Po2(3));
disp("Third Point: X: " + Po3(1) + " Y: " + Po3(2) + " Z: " + Po3(3));
disp("Fourth Point: X: " + Po4(1) + " Y: " + Po4(2) + " Z: " + Po4(3));

img = cam.cam.snapshot();
imshow(img);
axis on
hold on;
plot (201, 205, 'o', 'MarkerSize', 30, 'LineWidth', 2);
plot (102, 331, 'o', 'MarkerSize', 30, 'LineWidth', 2);
plot (524, 368, 'o', 'MarkerSize', 30, 'LineWidth', 2);
plot (510, 211, 'o', 'MarkerSize', 30, 'LineWidth', 2);

function Po = calcPositions(cam, pixelPos)

% camera pose returns the transformation from image to checkerboard
TimgChecker = cam.getCameraPose();
RimgChecker = TimgChecker(1:3, 1:3);
pimgChecker = TimgChecker(1:3, 4);

% calculated by hand
TbaseChecker = [0 1 0 75; 1 0 0 -100; 0 0 -1 0; 0 0 0 1;];

% position in the checker frame, takes care of TimgChecker so need for it
% in multiplication to find Po
Pchecker =  pointsToWorld(cam.params, RimgChecker, pimgChecker, pixelPos);
Pchecker = transpose(Pchecker);

ballWidth = 11; %distance to center of ball in mm
Pi = [Pchecker; ballWidth; 1;];

Po = TbaseChecker * Pi;
end

