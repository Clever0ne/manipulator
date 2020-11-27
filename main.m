clc; close all; clear;
robotRBT = loadrobot('universalUR10', 'DataFormat', 'column', ...
    'Gravity', [0 0 -9.81]);
robot = interactiveRigidBodyTree(robotRBT, 'ShowMarker', false);%, ...
    %'Configuration', [pi, pi, 0, pi, pi, 0]);
robotIKSolver = inverseKinematics('RigidBodyTree', robotRBT, ...
    'SolverAlgorithm', 'LevenbergMarquardt');
weights = [0, 0, 0, 1, 1, 1];
endEffector = robot.MarkerBodyName;
close;

addConfiguration(robot);
pointHome = robot.MarkerBodyPose;
point1st = trvec2tform([0.5, 0.5, 0.5]);
point2nd = trvec2tform([-0.5, 0.5, 0.5]);
point3nd = trvec2tform([-0.4, -0.4, 1.2]);
pointEnd = trvec2tform([0.3, -0.3, 1]);
          
rspTime_s = 0.001;
time_s = [0];

maxVel_mps = 2.0;
vel_mps = [0];

maxAcl_mps2 = 2.0;
acl_mps2 = [0];

points = {};
points{end + 1} = pointHome;
points{end + 1} = point1st;
points{end + 1} = point2nd;
points{end + 1} = point3nd;
points{end + 1} = pointEnd;

lengths_m = [];
for index = 1:(size(points, 2) - 1)
    currPoint = points{index};
    nextPoint = points{index + 1};
    
    currTrVec = tform2trvec(currPoint);
    nextTrVec = tform2trvec(nextPoint);
    
    lengths_m(end + 1) = norm(nextTrVec - currTrVec);
end

pointNumber = 1;
startAcceleration = true;
startDeceleration = false;
while (pointNumber <= size(points, 2))
    point = points{pointNumber};
    currPose = getTransform(robotRBT, robot.Configuration, endEffector);
    dsrdPose = point;
    
    currTrVec = tform2trvec(currPose);
    dsrdTrVec = tform2trvec(point);
    trVec = dsrdTrVec - currTrVec;
    
    distance_m = norm(trVec);
    error_m = vel_mps(end) * rspTime_s;
    if (distance_m <= error_m)
        pointNumber = pointNumber + 1;
        time_s(end);
        continue;
    end
    
    trajLen_m = distance_m;
    for index = pointNumber:(size(points, 2) - 1)
        trajLen_m = trajLen_m + lengths_m(index);
    end
    
    brkLen_m = ((vel_mps(end) + acl_mps2(end) * rspTime_s) ^ 2) / (2 * maxAcl_mps2);
    if (trajLen_m <= brkLen_m)
        startDeceleration = true;
        startAcceleration = false;
    end
    
    currTime_s = time_s(end) + rspTime_s;   
    
    currAcl_mps2 = 0;
    if (startAcceleration)
        currAcl_mps2 = maxAcl_mps2;
    end
    if (startDeceleration)
        currAcl_mps2 = -maxAcl_mps2;
    end
    
    currVel_mps = vel_mps(end) + (acl_mps2(end) + currAcl_mps2) * rspTime_s / 2;
    if (currVel_mps >= maxVel_mps)
        currVel_mps = maxVel_mps;
        currAcl_mps2 = 0;
        startAcceleration = false;
    end
    if (currVel_mps <= 0)
        currVel_mps = 0;
        currAcl_mps2 = 0;
        startDeceleration = false;
    end
    
    offset_m = (vel_mps(end) + currVel_mps) * rspTime_s / 2;
    currRotM = tform2rotm(currPose);
    nextTrVec = currTrVec + trVec * offset_m / distance_m;
    nextPose = trvec2tform(nextTrVec) * rotm2tform(currRotM);
    
    currConfig = robot.Configuration;
    nextConfig = robotIKSolver(endEffector, nextPose, weights, currConfig);
    robot.Configuration = nextConfig;
    addConfiguration(robot);
    %pause(rspTime_s);
    
    time_s(end + 1) = currTime_s;
    vel_mps(end + 1) = currVel_mps;
    acl_mps2(end + 1) = currAcl_mps2;
end

jointVel_radps = [];
currConfig = robot.StoredConfigurations(:, 1);
showFigure(robot);
for index = 1:size(robot.StoredConfigurations, 2)
    prevConfig = currConfig;
    currConfig = robot.StoredConfigurations(:, index);
    jointVel_radps(:, end + 1) = (currConfig - prevConfig) / rspTime_s;
    robot.Configuration = currConfig;
    currPose = robot.MarkerBodyPose;
    plot3(currPose(1, 4), currPose(2, 4), currPose(3, 4), 'b.');
    pause(rspTime_s);
end

figure;
plot(time_s, jointVel_radps(1, :), 'r',...
     time_s, jointVel_radps(2, :), 'g',...
     time_s, jointVel_radps(3, :), 'b',...
     time_s, jointVel_radps(4, :), 'c',...
     time_s, jointVel_radps(5, :), 'm',...
     time_s, jointVel_radps(6, :), 'k');
 axis([time_s(1), time_s(end), -10, 10]);
