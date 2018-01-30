function AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, LandMarksComputed)
% For Input and Output specifications refer to the project pdf

import gtsam.*
% Refer to Factor Graphs and GTSAM Introduction
% https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
% and the examples in the library in the GTSAM toolkit. See folder
% gtsam_toolbox/gtsam_examples
options.triangle = false;
options.nrCameras = 20;
options.showImages = false;

% iSAM Options
options.hardConstraint = false;
options.pointPriors = false;
options.batchInitialization = true;
options.reorderInterval = 10;
options.alwaysRelinearize = false;

% Display Options
options.saveDotFile = false;
options.printStats = false;
options.drawInterval = 5;
options.cameraInterval = 1;
options.drawTruePoses = false;
options.saveFigures = false;
options.saveDotFiles = false;
%%
%[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize);
intrinsic = Cal3_S2(K(1,1), K(2,2), K(1,2), K(1,3), K(2,3));
[~,n] = size(DetAll);
% init_1 = AllPosesComputed(1,:);
% init_2 = AllPosesComputed(2,:);
% T = init_1(1,1:3)';
% R = quat2rotm(init_1(1,4:7));
% pose_1 = Pose3(Rot3(R'), Point3(-R'*T));
% 
% T = init_2(1,1:3)';
% R = quat2rotm(init_2(1,4:7));
% pose = Pose3(Rot3(R'), Point3(-R'*T));
%% Initialize iSAM
params = gtsam.ISAM2Params;
if options.alwaysRelinearize
    params.setRelinearizeSkip(1);
end
isam = ISAM2(params);
%% Set Noise parameters
noiseModels.pose = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
%noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.001 0.001 0.001 0.1 0.1 0.1]');
noiseModels.odometry = noiseModel.Diagonal.Sigmas([0.005 0.005 0.005 0.0002 0.0002 0.0002]');
noiseModels.point = noiseModel.Isotropic.Sigma(3, 0.1);
noiseModels.measurement = noiseModel.Isotropic.Sigma(2, 1.0);

%% Add constraints/priors
% TODO: should not be from ground truth!
newFactors = NonlinearFactorGraph;
initialEstimates = Values;
AllPosesComputed = [0.0827   -0.7172    1.2074    0.1319   -0.9790    0.0864   -0.1289;...
                    0.0828   -0.7201    1.2053    0.1329   -0.9788    0.0877   -0.1289];
                    
for i=1:2
    ii = symbol('x',i);
    init = AllPosesComputed(i,:);
    T = init(1,1:3)';
    R = quat2rotm(init(1,4:7));
    pose = Pose3(Rot3(R'), Point3(-R'*T));
    if i==1
        if options.hardConstraint % add hard constraint
            newFactors.add(NonlinearEqualityPose3(ii,pose));
        else
            newFactors.add(PriorFactorPose3(ii,pose, noiseModels.pose));
        end
    end
    initialEstimates.insert(ii,pose);
end
%% Add visual measurement factors from two first poses and initialize observed landmarks
tag_ids_from_slam = LandMarksComputed(:,1);
nextPoseIndex = 1;
for i=1:2
    ii = symbol('x',i);
    [numDetections,~] = size(DetAll{i});
    for k=1:numDetections
        tagId = DetAll{i}(k,1);
        j = DetAll{i}(k,2:end); 
        count = 1;
        for t = 1:4
            jj = symbol('l',tagId*10 + t);
             %graph.add(GenericProjectionFactorCal3_S2(Point2(j(1),j(2)), measurementNoise, poses{i}, currSymbols.bottomleft, intrinsic));
            newFactors.add(GenericProjectionFactorCal3_S2(Point2(j(count),j(count+1)), noiseModels.measurement, ii, jj, intrinsic));
            % TODO: initial estimates should not be from ground truth!
            if ~initialEstimates.exists(jj)
                ind = find(tag_ids_from_slam == tagId);
                initialEstimates.insert(jj, Point3([LandMarksComputed(ind,count + 1:count + 2),0]'));
            end
            if options.pointPriors % add point priors
                newFactors.add(PriorFactorPoint3(jj, Point3([LandMarksComputed(ind,count + 1:count + 2),0]'), noiseModels.point));
            end
            count = count + 2;
        end
    end
end
%% Update ISAM
newFactors.add(BetweenFactorPose3(symbol('x',1), symbol('x',2), Pose3, noiseModels.odometry));
if options.batchInitialization % Do a full optimize for first two poses
    batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initialEstimates);
    fullyOptimized = batchOptimizer.optimize();
    isam.update(newFactors, fullyOptimized);
else
    isam.update(newFactors, initialEstimates);
end
result = isam.calculateEstimate();
nextPoseIndex = nextPoseIndex + 1;

for i=nextPoseIndex:4
    ii = symbol('x',i);
    [numDetections,~] = size(DetAll{i});
    for k=1:numDetections
        tagId = DetAll{i}(k,1);
        j = DetAll{i}(k,2:end); 
        count = 1;
        for t = 1:4
            jj = symbol('l',tagId*10 + t);
            newFactors.add(GenericProjectionFactorCal3_S2(Point2(j(count),j(count+1)), noiseModels.measurement, ii, jj, intrinsic));
            % TODO: initial estimates should not be from ground truth!
            if ~initialEstimates.exists(jj)
                ind = find(tag_ids_from_slam == tagId);
                initialEstimates.insert(jj, Point3([LandMarksComputed(ind,count + 1:count + 2),0]'));
            end
            if options.pointPriors % add point priors
                newFactors.add(PriorFactorPoint3(jj, Point3([LandMarksComputed(ind,count + 1:count + 2),0]'), noiseModels.point));
            end
            count = count + 2;
        end
    end
    %% Update ISAM
    prevPose = result.at(symbol('x',nextPoseIndex-1));
    initialEstimates.insert(symbol('x',nextPoseIndex), prevPose.compose(odometry));
    newFactors.add(BetweenFactorPose3(symbol('x',nextPoseIndex-1), symbol('x',nextPoseIndex), Pose3, noiseModels.odometry));
    if options.batchInitialization % Do a full optimize for first two poses
        batchOptimizer = LevenbergMarquardtOptimizer(newFactors, initialEstimates);
        fullyOptimized = batchOptimizer.optimize();
        isam.update(newFactors, fullyOptimized);
    else
        isam.update(newFactors, initialEstimates);
    end
    result = isam.calculateEstimate();
    nextPoseIndex = nextPoseIndex + 1;
end
end
