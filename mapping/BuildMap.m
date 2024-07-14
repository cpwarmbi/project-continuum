data = load("wareHouse.mat");
scans = data.wareHouseScans;
maxLidarRange = 8;
gridResolution = 20;
mapObj = lidarscanmap(gridResolution,maxLidarRange);
for i = 1:numel(scans)
    isScanAccepted = addScan(mapObj,scans{i});
     if ~isScanAccepted
        continue;
     end
end
hFigMap = figure;
axMap = axes(Parent=hFigMap);
show(mapObj,Parent=axMap);
title(axMap,"Map of the Environment and Robot Trajectory")