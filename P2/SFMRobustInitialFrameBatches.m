%%
clear all
%m = matfile('..\DataMapping.mat');
m = matfile('..\DataSquare.mat');
%m = matfile('..\DataSlowCircle.mat');
%m = matfile('..\DataStraightLine.mat');
load('..\CalibParams.mat')
%%
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples

DetAll = m.DetAll;
%% Generate data
% measurementNoiseSigma = 1.0;
% pointNoiseSigma = 0.1;
% poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
measurementNoiseSigma = 0.0001;
%pointNoiseSigma = 0.01;
poseNoiseSigmas = [0.00001 0.00001 0.00001 0.001 0.001 0.001]';
% measurementNoiseSigma = 1.75;
% %pointNoiseSigma = 0.1;
% poseNoiseSigmas = [0.4 0.6 0.1 0.1 0.1 0.08]';
%% Find the max num tag
[~,n] = size(DetAll);
maxNumTag =0;
for i = 1 : n
    if isempty(DetAll{i})
        continue;
    end
    currentTagIds = DetAll{i}(:,1);
    biggestTagId = max(currentTagIds);
    if maxNumTag < biggestTagId
        maxNumTag = biggestTagId;
    end
end
%% Get the amount of data
pointSymbols = cell(maxNumTag,1);
poses = cell(n,1);
% Create graph container and add factors to it from GTSAM library
graph = NonlinearFactorGraph;
%% Create keys for Poses
for i = 1 : n
    poses{i} = symbol('x',i);
end
%% Intial Estimate
initialEstimate = Values;
%% Add Gaussian priors for a pose and a landmark to constrain the system
originTagId = 10;
symbols0 = struct('bottomleft',symbol('p',originTagId * 10 + 1),...
           'bottomright', symbol('p',originTagId * 10 + 2),...
           'topright', symbol('p',originTagId * 10 + 3),...
           'topleft', symbol('p',originTagId * 10 + 4));
pointSymbols{originTagId} = symbols0;
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
%% Get Initial Pose and landmark points estimate
%TODO(Rishabh Biyani): Skip frames until Tag 10 is no visible
ind_origin = [];
x3d = zeros(4,2);
x3d(:,1:2) = [0 0;TagSize 0; TagSize TagSize;0 TagSize];
currentFrame = 0;
while isempty(ind_origin)
    currentFrame = currentFrame + 1;
    if isempty(DetAll{currentFrame})
        continue;
    end
    first_detections = DetAll{currentFrame};
    tag_ids_first = first_detections(:,1);
    ind_origin = find(tag_ids_first == originTagId);
end
j = first_detections(ind_origin,2:end);
x2d = zeros(4,2);
x2d(:,1:2) = [j(1) j(2); j(3) j(4); j(5) j(6); j(7) j(8)];
%% get the myhomography and pose in the first frame
pr = [ 1 0 0;
       0 1 0 ];
p_camframe = (pr*(K \ [x2d'; ones(1,4)]))';
%H = myhomography(x3d, p_camframe);
H = homography(x3d',x2d');
[R, T] = getPose2(H,K);
%% 
% Remove the origin Tag Id detections
first_detections = [first_detections(1:ind_origin-1,:);first_detections(ind_origin + 1 : end,:)];
DetAll{currentFrame} = first_detections;
%% Add this prior mean to the pose in the first frame
priorMean = Pose3(Rot3(R'), Point3(-R'*T));
%priorMean = Pose3(Rot3(R), Point3(T));
graph.add(PriorFactorPose3(poses{currentFrame}, priorMean, posePriorNoise));
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,0.0001);
%% Add prior factors on the points in the first frame
graph.add(PriorFactorPoint3(symbols0.bottomleft, Point3(0,0,0), pointPriorNoise));
graph.add(PriorFactorPoint3(symbols0.bottomright, Point3(TagSize,0,0), pointPriorNoise));
graph.add(PriorFactorPoint3(symbols0.topright, Point3(TagSize,TagSize,0), pointPriorNoise));
graph.add(PriorFactorPoint3(symbols0.topleft, Point3(0,TagSize,0), pointPriorNoise));

% Add keys to initial estimate
point_j = Point3(0,0,0); 
point_j = point_j.retract(0.001*randn(3,1));
initialEstimate.insert(symbols0.bottomleft, point_j);
point_j = Point3(TagSize,0,0);
point_j = point_j.retract(0.001*randn(3,1));
initialEstimate.insert(symbols0.bottomright, point_j);
point_j = Point3(TagSize,TagSize,0);
point_j = point_j.retract(0.001*randn(3,1));
initialEstimate.insert(symbols0.topright, point_j);
point_j = Point3(0,TagSize,0);
point_j = point_j.retract(0.001*randn(3,1));
initialEstimate.insert(symbols0.topleft, point_j); 
%% Initialize some other variables
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
constraintNoise = noiseModel.Diagonal.Sigmas([0.0001; 0.0001; 0.0001]);
constraintNoise2 = noiseModel.Diagonal.Sigmas(0.001 * ones(6,1));
IdentityPose = Pose3(Rot3(eye(3)), Point3([0;0;0]));
intrinsic = Cal3_S2(K(1,1), K(2,2), K(1,2), K(1,3), K(2,3));
%% Populate 3D points for first frame detections
% TODO(Rishabh Biyani) Only take the myhomography out of first tag, populate
% rest for even the first frame using PNP
[numDetections,~] = size(DetAll{currentFrame});
for k=1:numDetections
    tagId = DetAll{currentFrame}(k,1);
    j = DetAll{currentFrame}(k,2:end);
    if isempty(pointSymbols{tagId})
        currSymbols = struct('bottomleft',symbol('p',tagId * 10 + 1),...
                             'bottomright', symbol('p',tagId * 10 + 2),...
                             'topright', symbol('p',tagId * 10 + 3),...
                             'topleft', symbol('p',tagId * 10 + 4));
    % Constrain between points
    graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.bottomright, Point3(TagSize,0,0), constraintNoise));
    graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.topleft, Point3(0,TagSize,0), constraintNoise));
    graph.add(BetweenFactorPoint3(currSymbols.bottomright, currSymbols.topright, Point3(0,TagSize,0), constraintNoise));
    graph.add(BetweenFactorPoint3(currSymbols.topleft, currSymbols.topright, Point3(TagSize,0,0), constraintNoise));
    graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.topright, Point3(TagSize,TagSize,0), constraintNoise));
    graph.add(BetweenFactorPoint3(currSymbols.bottomright, currSymbols.topleft, Point3(-TagSize,TagSize,0), constraintNoise));
    pointSymbols{tagId} = currSymbols;
    % Get the Tag 3D points using planar myhomography assumption
    %X = get3DPointsfromHomography(H, j, K);
    X = get3DPointFromH(j,H);
    X(3,:) = 0;
    point_j = Point3(X(:,1));
    initialEstimate.insert(currSymbols.bottomleft, point_j);
    point_j = Point3(X(:,2));
    initialEstimate.insert(currSymbols.bottomright, point_j);
    point_j = Point3(X(:,3));
    initialEstimate.insert(currSymbols.topright, point_j);
    point_j = Point3(X(:,4));
    initialEstimate.insert(currSymbols.topleft, point_j);          
    else
        currSymbols = pointSymbols{tagId};
    end
    % Projection factors
    graph.add(GenericProjectionFactorCal3_S2(Point2(j(1),j(2)), measurementNoise, poses{currentFrame}, currSymbols.bottomleft, intrinsic));
    graph.add(GenericProjectionFactorCal3_S2(Point2(j(3),j(4)), measurementNoise, poses{currentFrame}, currSymbols.bottomright, intrinsic))
    graph.add(GenericProjectionFactorCal3_S2(Point2(j(5),j(6)), measurementNoise, poses{currentFrame}, currSymbols.topright, intrinsic))
    graph.add(GenericProjectionFactorCal3_S2(Point2(j(7),j(8)), measurementNoise, poses{currentFrame}, currSymbols.topleft, intrinsic))
end
pose_i = Pose3(Rot3(R'), Point3(-R'*T));
%lastTranslation = -R'*T;
%last_pose = pose_i;
%pose_i = Pose3(Rot3(R), Point3(T));
initialEstimate.insert(poses{currentFrame}, pose_i);
%% Iterate over previous frames if Tag 10 was not seen
currenFrameMakeone = currentFrame;
tag_ids_deplete = tag_ids_first;
while (currenFrameMakeone ~= 1)
    previousFrame = currenFrameMakeone - 1;
    % Get the common tags between the previous frame and the current frame
    prevDetections = DetAll{previousFrame};
    tag_ids_prev = prevDetections(:,1);
    [common_tag_ids, uncommon_tag_ids] = getCommonTags(tag_ids_deplete, tag_ids_prev);
    [numDetections,~] = size(DetAll{previousFrame});
    numcommon = size(common_tag_ids,1);
    numuncommon = size(uncommon_tag_ids,1);
    x3d_all = zeros(numcommon * 4, 3);
    x2d_all = zeros(numcommon * 4,2);
    % Estimate the pose for this frame using PNP
    point_ind = 1;
    for t = 1 : numcommon
        common_tag_id = common_tag_ids(t);
        ind_tag = find(tag_ids_prev == common_tag_id);
        j = prevDetections(ind_tag,2:end);
        x2d(:,1:2) = [j(1) j(2); j(3) j(4); j(5) j(6); j(7) j(8)];
        x3d(:,1:3) = [(initialEstimate.at(pointSymbols{common_tag_id}.bottomleft).x),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.bottomleft).y),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.bottomleft).z);...
                        (initialEstimate.at(pointSymbols{common_tag_id}.bottomright).x),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.bottomright).y),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.bottomright).z);...
                        (initialEstimate.at(pointSymbols{common_tag_id}.topright).x),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.topright).y),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.topright).z);...
                        (initialEstimate.at(pointSymbols{common_tag_id}.topleft).x),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.topleft).y),...
                        (initialEstimate.at(pointSymbols{common_tag_id}.topleft).z)];
        currSymbols = pointSymbols{common_tag_id};
        % Add Generic projection factor for point and this pose
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(1),j(2)), measurementNoise, poses{previousFrame}, currSymbols.bottomleft, intrinsic));
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(3),j(4)), measurementNoise, poses{previousFrame}, currSymbols.bottomright, intrinsic))
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(5),j(6)), measurementNoise, poses{previousFrame}, currSymbols.topright, intrinsic))
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(7),j(8)), measurementNoise, poses{previousFrame}, currSymbols.topleft, intrinsic))
        x2d_all(point_ind:point_ind+3,:) = x2d;
        x3d_all(point_ind:point_ind+3,:) = x3d;
        point_ind = point_ind + 4;
    end     
     H = homography(x3d_all(:,1:2)',x2d_all');
    [R, T] = getPose2(H,K);
    for k=1:numuncommon
        uncommon_tag_id = uncommon_tag_ids(k);
        ind_tag = find(tag_ids_prev == uncommon_tag_id);
        j = nextDetections(ind_tag,2:end);
        if isempty(pointSymbols{uncommon_tag_id})
            currSymbols = struct('bottomleft',symbol('p',uncommon_tag_id * 10 + 1),...
                                 'bottomright', symbol('p',uncommon_tag_id * 10 + 2),...
                                 'topright', symbol('p',uncommon_tag_id * 10 + 3),...
                                 'topleft', symbol('p',uncommon_tag_id * 10 + 4));
            % Constrain between points
            graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.bottomright, Point3(TagSize,0,0), constraintNoise));
            graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.topleft, Point3(0,TagSize,0), constraintNoise));
            graph.add(BetweenFactorPoint3(currSymbols.bottomright, currSymbols.topright, Point3(0,TagSize,0), constraintNoise));
            graph.add(BetweenFactorPoint3(currSymbols.topleft, currSymbols.topright, Point3(TagSize,0,0), constraintNoise));
            graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.topright, Point3(TagSize,TagSize,0), constraintNoise));
            graph.add(BetweenFactorPoint3(currSymbols.bottomright, currSymbols.topleft, Point3(-TagSize,TagSize,0), constraintNoise));

            pointSymbols{uncommon_tag_id} = currSymbols;
            % Give a Initial Estimate for new Landmark points
%            [X] = get3DPointsFromPoseAndPixel(R', -R'*T, j, K, scale);
            %[X] = get3DPointsFromPoseAndPixel(R, T, j, K, scale);
            X = get3DPointFromH(j,H);
            X(3,:) = 0;
            point_j = Point3(X(:,1));
            initialEstimate.insert(currSymbols.bottomleft, point_j);
            point_j = Point3(X(:,2));
            initialEstimate.insert(currSymbols.bottomright, point_j);
            point_j = Point3(X(:,3));
            initialEstimate.insert(currSymbols.topright, point_j);
            point_j = Point3(X(:,4));
            initialEstimate.insert(currSymbols.topleft, point_j);
        else
            currSymbols = pointSymbols{tagId};
        end
        
        % Generic projection Factor for these points and the current pose
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(1),j(2)), measurementNoise, poses{previousFrame}, currSymbols.bottomleft, intrinsic));
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(3),j(4)), measurementNoise, poses{previousFrame}, currSymbols.bottomright, intrinsic))
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(5),j(6)), measurementNoise, poses{previousFrame}, currSymbols.topright, intrinsic))
        graph.add(GenericProjectionFactorCal3_S2(Point2(j(7),j(8)), measurementNoise, poses{previousFrame}, currSymbols.topleft, intrinsic))
    end
    % Add Initial Estimate for the Pose of the current frame
    pose_i = Pose3(Rot3(R'), Point3(-R'*T));
    %pose_i = Pose3(Rot3(R), Point3(T));
    initialEstimate.insert(poses{previousFrame}, pose_i);

    % Add Between pose constant factor?
     graph.add(BetweenFactorPose3(poses{previousFrame}, poses{currenFrameMakeone}, IdentityPose, constraintNoise2));
    % Change the tag_ids
    tag_ids_deplete = tag_ids_prev;
    % decrement frame number
    currenFrameMakeone = previousFrame;
end
%% iterate over rest of the frames
skipped = false;
batchsize = 100;
frame_0 = currentFrame + 1;
frame_n = frame_0 + batchsize;
while (frame_0 <= n)
    for i=frame_0:frame_n
        % Get the common tags between the current frame and the previous frame
        if isempty(DetAll{i})
            skipped = true;
            continue;
        end
        nextDetections = DetAll{i};
        tag_ids_next = nextDetections(:,1);
        [common_tag_ids, uncommon_tag_ids] = getCommonTags(tag_ids_first, tag_ids_next);
        [numDetections,~] = size(DetAll{i});
        numcommon = size(common_tag_ids,1);
        numuncommon = size(uncommon_tag_ids,1);
        x3d_all = zeros(numcommon * 4, 3);
        x2d_all = zeros(numcommon * 4,2);
        % Estimate the pose for this frame using PNP
        point_ind = 1;
        for t = 1 : numcommon
            common_tag_id = common_tag_ids(t);
            ind_tag = find(tag_ids_next == common_tag_id);
            j = nextDetections(ind_tag,2:end);
            x2d(:,1:2) = [j(1) j(2); j(3) j(4); j(5) j(6); j(7) j(8)];
            x3d(:,1:3) = [(initialEstimate.at(pointSymbols{common_tag_id}.bottomleft).x),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.bottomleft).y),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.bottomleft).z);...
                            (initialEstimate.at(pointSymbols{common_tag_id}.bottomright).x),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.bottomright).y),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.bottomright).z);...
                            (initialEstimate.at(pointSymbols{common_tag_id}.topright).x),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.topright).y),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.topright).z);...
                            (initialEstimate.at(pointSymbols{common_tag_id}.topleft).x),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.topleft).y),...
                            (initialEstimate.at(pointSymbols{common_tag_id}.topleft).z)];
            currSymbols = pointSymbols{common_tag_id};
            % Add Generic projection factor for point and this pose
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(1),j(2)), measurementNoise, poses{i}, currSymbols.bottomleft, intrinsic));
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(3),j(4)), measurementNoise, poses{i}, currSymbols.bottomright, intrinsic))
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(5),j(6)), measurementNoise, poses{i}, currSymbols.topright, intrinsic))
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(7),j(8)), measurementNoise, poses{i}, currSymbols.topleft, intrinsic))
            x2d_all(point_ind:point_ind+3,:) = x2d;
            x3d_all(point_ind:point_ind+3,:) = x3d;
            point_ind = point_ind + 4;
        end
    %     % TODO(Rishabh Biyani) if size(x2d_all,1) < 6 switch to myhomography
    %     p_camframe = (pr*(K \ [x2d_all'; ones(1,size(x2d_all,1))]))';
    %     H = myhomography(x3d_all, p_camframe);
    %     [R, T] = getPose(H);

         H = homography(x3d_all(:,1:2)',x2d_all');
        [R, T] = getPose2(H,K);
        C = -R' * T;
        if (C(3) < 0)
           var = 1;
        end
        %[R,T,~,~]= efficient_pnp(x3d_all,x2d_all,K);
        %P = K * [eye(3) zeros(3,1)] * [R' -R'*T;0 0 0 1];
        P = K * [eye(3) zeros(3,1)] * [R T;0 0 0 1];
        x3d_h = [x3d_all(1,:) 1];
        img_coords = P * x3d_h';
        scale = img_coords(3);
        for k=1:numuncommon
            uncommon_tag_id = uncommon_tag_ids(k);
            ind_tag = find(tag_ids_next == uncommon_tag_id);
            j = nextDetections(ind_tag,2:end);
            if isempty(pointSymbols{uncommon_tag_id})
                currSymbols = struct('bottomleft',symbol('p',uncommon_tag_id * 10 + 1),...
                                     'bottomright', symbol('p',uncommon_tag_id * 10 + 2),...
                                     'topright', symbol('p',uncommon_tag_id * 10 + 3),...
                                     'topleft', symbol('p',uncommon_tag_id * 10 + 4));
                % Constrain between points
                graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.bottomright, Point3(TagSize,0,0), constraintNoise));
                graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.topleft, Point3(0,TagSize,0), constraintNoise));
                graph.add(BetweenFactorPoint3(currSymbols.bottomright, currSymbols.topright, Point3(0,TagSize,0), constraintNoise));
                graph.add(BetweenFactorPoint3(currSymbols.topleft, currSymbols.topright, Point3(TagSize,0,0), constraintNoise));
                graph.add(BetweenFactorPoint3(currSymbols.bottomleft, currSymbols.topright, Point3(TagSize,TagSize,0), constraintNoise));
                graph.add(BetweenFactorPoint3(currSymbols.bottomright, currSymbols.topleft, Point3(-TagSize,TagSize,0), constraintNoise));

                pointSymbols{uncommon_tag_id} = currSymbols;
                % Give a Initial Estimate for new Landmark points
    %            [X] = get3DPointsFromPoseAndPixel(R', -R'*T, j, K, scale);
                %[X] = get3DPointsFromPoseAndPixel(R, T, j, K, scale);
                X = get3DPointFromH(j,H);
                X(3,:) = 0;
                %X = X / 10;
                %X = X ./ X(3,:);
                % X(3,:) = X(3,:)/10;
                point_j = Point3(X(:,1));
                initialEstimate.insert(currSymbols.bottomleft, point_j);
                point_j = Point3(X(:,2));
                initialEstimate.insert(currSymbols.bottomright, point_j);
                point_j = Point3(X(:,3));
                initialEstimate.insert(currSymbols.topright, point_j);
                point_j = Point3(X(:,4));
                initialEstimate.insert(currSymbols.topleft, point_j);
            else
                currSymbols = pointSymbols{tagId};
            end

            % Generic projection Factor for these points and the current pose
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(1),j(2)), measurementNoise, poses{i}, currSymbols.bottomleft, intrinsic));
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(3),j(4)), measurementNoise, poses{i}, currSymbols.bottomright, intrinsic))
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(5),j(6)), measurementNoise, poses{i}, currSymbols.topright, intrinsic))
            graph.add(GenericProjectionFactorCal3_S2(Point2(j(7),j(8)), measurementNoise, poses{i}, currSymbols.topleft, intrinsic))
        end
        % Add Initial Estimate for the Pose of the current frame
        pose_i = Pose3(Rot3(R'), Point3(-R'*T));
        %pose_i = Pose3(Rot3(R), Point3(T));
        currentTranslation = -R'*T;
        initialEstimate.insert(poses{i}, pose_i);
        %pose_i = Pose3(Rot3(R), Point3(T));
    %     if norm(lastTranslation - currentTranslation) > 1
    %         initialEstimate.insert(poses{i}, last_pose);
    %     else
    %         initialEstimate.insert(poses{i}, pose_i);
    %         last_pose = pose_i;
    %     end
        % Add Between pose constant factor?
        if ~skipped
            graph.add(BetweenFactorPose3(poses{i-1}, poses{i}, IdentityPose, constraintNoise2));
        else
            graph.add(BetweenFactorPose3(poses{i-2}, poses{i}, IdentityPose, constraintNoise2));
            skipped  = false;
        end
        % Change the tag_ids
        tag_ids_first = tag_ids_next;
    end
        %% Optimize
        optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
        result = optimizer.optimizeSafely();
        intialEstimate = result;
        frame_0 = frame_0 + batchsize + 1;
        frame_n = frame_0 + batchsize;
        if frame_n > n
            frame_n = n;
        end
end
%% Fine grain optimization, allowing user to iterate step by step
% parameters = LevenbergMarquardtParams;
% parameters.setlambdaInitial(0.1);
% parameters.setVerbosityLM('trylambda');
% 
% optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);
% 
% for i=1:5
%     optimizer.iterate();
% end
% result = optimizer.values();
%% Get Values
pointsComputed = [];
for i = 1 : maxNumTag
    if ~isempty(pointSymbols{i})
        pointsComputed = [pointsComputed;[result.at(pointSymbols{i}.bottomleft).x, result.at(pointSymbols{i}.bottomleft).y]];
        pointsComputed = [pointsComputed;[result.at(pointSymbols{i}.bottomright).x, result.at(pointSymbols{i}.bottomright).y]];
        pointsComputed = [pointsComputed;[result.at(pointSymbols{i}.topright).x, result.at(pointSymbols{i}.topright).y]];
        pointsComputed = [pointsComputed;[result.at(pointSymbols{i}.topleft).x, result.at(pointSymbols{i}.topleft).y]];
    end
end
%%
LandMarksComputed = [];
for i = 1 : maxNumTag
    if ~isempty(pointSymbols{i})
        LandMarksComputed = [LandMarksComputed;[i, result.at(pointSymbols{i}.bottomleft).x, result.at(pointSymbols{i}.bottomleft).y,...
                             result.at(pointSymbols{i}.bottomright).x, result.at(pointSymbols{i}.bottomright).y,...
                             result.at(pointSymbols{i}.topright).x, result.at(pointSymbols{i}.topright).y,...
                             result.at(pointSymbols{i}.topleft).x, result.at(pointSymbols{i}.topleft).y]];
    end
end
%% Get Poses
num_poses = n;
AllPosesComputed = zeros(num_poses,3);
%LandmarksComputed = zeros(numLandmarks,3);
for i = 1 : num_poses
    if isempty(DetAll{i})
        continue;
    end
    AllPosesComputed(i,:) = [result.at(poses{i}).x result.at(poses{i}).y result.at(poses{i}).z];
end
%% Plot
t = size(pointsComputed,1);
scatter3(pointsComputed(:,1),pointsComputed(:,2),zeros(t,1), 'bo')
%% Plot Poses
hold on
scatter3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3), 'ro')