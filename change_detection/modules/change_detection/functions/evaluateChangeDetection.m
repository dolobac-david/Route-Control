% Evaluation of change detection algorithm.
% Divide points of both point clouds to change (flag 1) and no
% change (flag 2) classes and to ground true and prediction classes.
function [sensitivity,specificity,FPR,valuesGT,valuesPredict,accuracy] = evaluateChangeDetection(route1,route2,ptCloudFixedCropped,pointsIdx1SurfaceChange,ptCloudMoved,pointsIdx2SurfaceChange)

% Original point cloud.

% Ground true values.
load("data\evaluation_ground_true\"+route1+"_and_" ...
    +route2+"\ROIs"+ ...
    route1+".mat","objectsROIsOriginal");
idxGTChangeFixedCropped = selectGTPoints(objectsROIsOriginal, ptCloudFixedCropped,0);
valuesGTFixedCropped = zeros(ptCloudFixedCropped.Count,1);
valuesGTFixedCropped(idxGTChangeFixedCropped) = 1;

% Predicted values.
idxPredictChangeFixedCropped = pointsIdx1SurfaceChange';
valuesPredictFixedCropped = zeros(ptCloudFixedCropped.Count,1);
valuesPredictFixedCropped(idxPredictChangeFixedCropped) = 1;

% Current location point cloud.

% Ground true values.
load("data\evaluation_ground_true\"+route1+"_and_" ...
    +route2+"\ROIs"+ ...
    route2+".mat","objectsROIsCurrent");
idxGTChangeMoved = selectGTPoints(objectsROIsCurrent, ptCloudMoved,0);
valuesGTMoved = zeros(ptCloudMoved.Count,1);
valuesGTMoved(idxGTChangeMoved) = 1;

% Predicted values.
idxPredictChangeMoved = pointsIdx2SurfaceChange';
valuesPredictMoved = zeros(ptCloudMoved.Count,1);
valuesPredictMoved(idxPredictChangeMoved) = 1;

% Fuse both point clouds for single evaluation.
valuesGT = [valuesGTFixedCropped; valuesGTMoved];
valuesPredict = [valuesPredictFixedCropped; valuesPredictMoved];

TP = nnz(valuesGT == 1 & valuesPredict == 1);
FP = nnz(valuesGT == 0 & valuesPredict == 1);
FN = nnz(valuesGT == 1 & valuesPredict == 0);
TN = nnz(valuesGT == 0 & valuesPredict == 0);

P = TP + FN;
N = TN + FP;

% TPR (true positive rate)
sensitivity = TP/P;
% TNR (true negative rate)
specificity = TN/N;
% False positive rate
FPR = FP/N;
% precision = TP/(TP+FP);
accuracy = (TP+TN)/(P+N);
end