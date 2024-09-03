% Load the JSON data
filename = 'lidar_scans.json';
data = jsondecode(fileread(filename));

% Parameters
maxLidarRange = 8000; % Maximum range in mm (8 meters)
gridResolution = 20;  % Resolution of the grid map
numScans = length(data);  % Number of scan entries to process

% Initialize the map object
mapObj = lidarscanmap(gridResolution, maxLidarRange);

% Loop through the first numScans entries
for i = 1:numScans
    % Initialize the map object for each scan
    mapObj = lidarscanmap(gridResolution, maxLidarRange);
    
    % Extract scan data
    scan_data = data(i).scan_data;
    
    % Convert the polar coordinates (angle, distance) to Cartesian (x, y)
    angles = deg2rad(0:359); % Angles in radians
    distances = scan_data; % Distances in mm
    x = distances .* cos(angles);
    y = distances .* sin(angles);
    
    % Create lidarScan object
    currScan = lidarScan(distances, angles);
    
    % Add the scan to the map
    isScanAccepted = addScan(mapObj, currScan);
    if ~isScanAccepted
        continue;
    end
end

% Display the map
hFigMap = figure;
axMap = axes(Parent=hFigMap);
show(mapObj, 'Parent', axMap);
title(axMap, "2D Point Cloud Map of the First 5 Scans");