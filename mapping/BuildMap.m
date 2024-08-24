% Load the JSON file
filename = 'scans.json';
fid = fopen(filename); 
raw = fread(fid, inf); 
str = char(raw'); 
fclose(fid); 
data = jsondecode(str);

% Check the structure of the data
if isfield(data, 'scan_data')
    scanData = data.scan_data;
else
    error('The JSON file does not contain "scan_data". Please check the file structure.');
end

% Assuming each 'scan_data' entry is a row vector of 360 elements representing a lidar scan
numScans = length(scanData);
angles = linspace(0, 2*pi, 360); % 360 angles evenly spaced between 0 and 2*pi

% Preallocate arrays for x and y coordinates of the point cloud
x_coords = zeros(1, 360 * numScans);
y_coords = zeros(1, 360 * numScans);

% Populate the preallocated arrays
for i = 1:numScans
    % Adjust based on the type of scanData
    if iscell(scanData)
        distances = scanData{i};  % Cell array indexing
    elseif isnumeric(scanData)
        distances = scanData(i, :);  % Numeric array indexing
    elseif isstruct(scanData)
        distances = scanData(i).scan_data;  % Structure array indexing
    else
        error('Unsupported data type for scanData.');
    end
    
    % Check the length of the distances array
    if length(distances) < 360
        % Pad the array with zeros (or NaNs) if it has fewer than 360 elements
        distances = [distances, zeros(1, 360 - length(distances))];
    elseif length(distances) > 360
        % Truncate the array if it has more than 360 elements (just in case)
        distances = distances(1:360);
    end
    
    % Convert distances from mm to meters
    distances = distances / 1000; 
    
    % Calculate the start index for the current scan in the preallocated arrays
    startIdx = (i-1) * 360 + 1;
    endIdx = i * 360;
    
    % Convert polar coordinates to Cartesian coordinates
    x_coords(startIdx:endIdx) = distances .* cos(angles);
    y_coords(startIdx:endIdx) = distances .* sin(angles);
end

% Plot the 2D point cloud
figure;
scatter(x_coords, y_coords, 1, 'filled');
title('2D Lidar Point Cloud Map');
xlabel('X (meters)');
ylabel('Y (meters)');
axis equal;
grid on;
