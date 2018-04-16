%% This file reads data from just pose of results formated in the following way:
% position: 
%   x: 0.067956417799
%   y: -0.0385386645794
%   z: -0.0911143794656
% orientation: 
%   x: 3.61251472363e-10
%   y: -1.51125867376e-08
%   z: 5.405129668e-11
%   w: 1.0
% ---
close all
clear
clc

%% Paramsters you can change
sequenceNumber = '08'; %change this to which sequence you want analyzed
plotInRealTime = 1; %used to plot errors and position in real time to see where they stem from while moving
%rotatation needed for points, changes per data set
translateAndRotate = 1; %used to apply translation and rotation
theta = -.5768; %radians
rotationOn = 1; %turn on/off rotation. Useful for first determining rotation based on results below before running function

resultsPath = 'Results/'; %change this depending on where the results are
groundTruthPath = 'GroundTruth/'; %change this depending on where the Ground Truth are
picturesPath = 'Pictures/'; %change this depending on where you want to save results
resultsFile = [resultsPath 'sequence' sequenceNumber '.txt']; %name of results file
groundTruthFile = [groundTruthPath sequenceNumber '.txt']; %name of ground truth file

R = [cos(theta) -sin(theta); sin(theta) cos(theta)]; %rotation matrix

fileID = fopen(resultsFile, 'r');
a = fscanf(fileID, '%s'); %read data with no spaces

%data is from echoed rostopic bag file of /pose/pose
TEXT_POSE = 'position:';
finderPose = strfind(a, TEXT_POSE); %beginning of position data
TEXT_ORIENTATION = 'orientation:';
finderOrientation = strfind(a, TEXT_ORIENTATION); %beginning of orientation data

%extract x, y, z values from results file
count = 1;
i = finderPose(1);
while i <= length(a)
    j = i + length(TEXT_POSE);
    while j < finderOrientation(count)
        %find where x, y, and z start/end in text file per measurement
        if a(j:j+length('x:')-1) == 'x:'
            beforeX = j-1;
            j = j + length('x:');
            afterX = j;
        elseif a(j:j+length('y:')-1) == 'y:'
            beforeY = j-1;
            j = j + length('y:');
            afterY = j;
        elseif a(j:j+length('z:')-1) == 'z:'
            beforeZ = j-1;
            j = j + length('z:');
            afterZ = j;
        end
        j = j+1;
    end
    %bring in values as numbers instead of strings
    position(count, 1) = str2num(a(afterX:beforeY)); %x
    position(count, 2) = str2num(a(afterY:beforeZ)); %y
    position(count, 3) = str2num(a(afterZ:finderOrientation(count)-1)); %z
    count = count + 1;
    %only incremnt count when there is a result left to be extracted,
    %otherwise saturate to end of file+1 to end while loop
    if count <= length(finderPose)
        i = finderPose(count);
    else
        i = length(a)+1;
    end
end
fclose(fileID);

fileID = fopen(groundTruthFile, 'r');
b = fscanf(fileID, '%f'); %read data as floats
temp = reshape(b,[12 length(b)/12])';
groundTruth = [temp(:,4) temp(:,8) temp(:,12)];
fclose(fileID);

deltaLength = length(position)-length(groundTruth);
if translateAndRotate == 1
    deltaLength = length(position)-length(groundTruth);
    %apply rotation to true x and y coordinates (x and z in file)
    i = 1;
    while i <= length(position) && rotationOn == 1
        temp = [position(i,1) position(i,3)]*R;
        convertPosition(i,1) = -temp(1); %negative needed to flip x
        convertPosition(i,2) = temp(2);
        i = i + 1;
    end
    %results file has extra points so remove them and offset points to start at
    %same position
    %start position at the same point for results and ground truth
    Xoffset = convertPosition(deltaLength+1,1);
    Zoffset = convertPosition(deltaLength+1,2);
    convertPosition(:,1) = convertPosition(:,1) - Xoffset;
    convertPosition(:,2) = convertPosition(:,2) - Zoffset;
    %remove excees points in results
    convertPosition = convertPosition(deltaLength+1:end,:);
else
    convertPosition(:,1) = position(deltaLength+1:end,1);
    convertPosition(:,2) = position(deltaLength+1:end,3);
end

figure
plot(groundTruth(:,1), groundTruth(:,3))
title('Ground Truth')
axis([-500 500 -100 400])
saveas(gcf, [picturesPath sequenceNumber 'GroundTruth.png'])
figure
plot(convertPosition(:,1), convertPosition(:,2))
title('Results')
axis([-500 500 -100 400])
saveas(gcf, [picturesPath sequenceNumber 'Results.png'])
figure
hold on
plot(groundTruth(:,1), groundTruth(:,3))
plot(convertPosition(:,1), convertPosition(:,2))
title('Ground Truth vs. Results')
legend('Ground Truth', 'Results')
axis([-500 500 -100 400])
x = [0.45 0.515];
y = [0.2 0.27];
annotation('textarrow',x,y,'String','Starting Position')
x = [0.45 0.5125];
y = [0.88 0.78];
annotation('textarrow',x,y,'String','Ending Position')
saveas(gcf, [picturesPath sequenceNumber 'GTvR.png'])


for i = 2:length(groundTruth)
    %calculate translation error
    dxP = (convertPosition(i-1,1)-convertPosition(i,1));
    dyP = (convertPosition(i-1,2)-convertPosition(i,2));
    translationResults(i) = sqrt(dxP^2 + dyP^2);
    dxG = (groundTruth(i,1)-groundTruth(i-1,1));
    dyG = (groundTruth(i,3)-groundTruth(i-1,3));
    translationGroundTruth(i) = sqrt(dxG^2 + dyG^2);
    translationError(i) = translationResults(i) - translationGroundTruth(i);
    translationPercentError(i) = sum(translationError)/length(translationError);
    
    %calculate rotation error
    rotationResults(i) = atan2(convertPosition(i,2)-convertPosition(i-1,2),...
        convertPosition(i,1)-convertPosition(i-1,1));
    rotationGroundTruth(i) = atan2(groundTruth(i,3)-groundTruth(i-1,3),...
        groundTruth(i,1)-groundTruth(i-1,1));
    rotationError(i) = minimizedAngle(rotationGroundTruth(i) - rotationResults(i));
end
finalRotationError = sum(rotationError)/length(rotationError);
absolutePositionError = sqrt((convertPosition(end,1)-groundTruth(end,1))^2 +...
    (convertPosition(end,2)-groundTruth(end,3))^2);

xAx = [1:length(translationError)];
figure
subplot(2,1,1)
plot(xAx, translationError)
title('Translation Error')
subplot(2,1,2)
plot(xAx, rotationError)
title('Rotation Error')
saveas(gcf, [picturesPath sequenceNumber 'ErrorsInTime.png'])

disp(['Translation Percent Error = ' num2str(translationPercentError(end))])
disp(['Rotation Error = ' num2str(finalRotationError)])
%disp(['Rotation Error = ' num2str(rotationError2(end))])
%disp(['Rotation Error = ' num2str(rotationError3(end))])
disp(['Absolute Position Error = ' num2str(absolutePositionError)])

figure
if plotInRealTime == 1
    for i = 1:20:length(translationError)
        ax(1) = subplot(2,2,1);
        plot(xAx(1:i), translationError(1:i), 'b')
        title('Translation Error')
        ax(2) = subplot(2,2,3);
        plot(xAx(1:i), rotationError(1:i), 'b')
        title('Rotation Error')
        ax(3) = subplot(2,2,[2 4]);
        hold on
        plot(groundTruth(1:i,1), groundTruth(1:i,3), 'b')
        plot(convertPosition(1:i,1), convertPosition(1:i,2), 'r')
        title('Ground Truth vs. Results')
        legend('Ground Truth', 'Results')
        drawnow
        pause(.02)
    end
else
    ax(1) = subplot(2,2,1);
    plot(xAx(1:i), translationError, 'b')
    title('Translation Error')
    ax(2) = subplot(2,2,3);
    plot(xAx(1:i), rotationError, 'b')
    title('Rotation Error')
    ax(3) = subplot(2,2,[2 4]);
    hold on
    plot(groundTruth(:,1), groundTruth(:,3), 'b')
    plot(convertPosition(:,1), convertPosition(:,2), 'r')
    title('Ground Truth vs. Results')
    legend('Ground Truth', 'Results')
end
saveas(gcf, [picturesPath sequenceNumber 'ErrorsAndPositionInTime.png'])


function angle = minimizedAngle(angle)

while angle < -pi
    angle = angle + 2*pi;
end

while angle >= pi
    angle = angle - 2*pi;
end
end
