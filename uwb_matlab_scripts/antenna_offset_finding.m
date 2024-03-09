% Specify the number of samples to average when determining the distance
% matrix
num_readings = 3;

% Specify the device names
% Note, you can get the device names using "blelist" command of MATLAB
device_name_1 = "E10FBE416497"; % device 1 is assumed to be the initiator
device_name_2 = "DC7BE4B47137";
device_name_3 = "F982750A320E";
device_name_4 = "CDBFFDF40783";


% Device Addresses
"D8B3991DE667"; % 028D, 64mm, 57, 60,
"EB95E21BC1E0"; % C326, 75mm, 90, 80,
"E464B7171265"; % C702, 86mm, 80, 90,
"F982750A320E"; % 1032, 108mm ,114, 109
"DC7BE4B47137"; % 85A7, 84mm, 104, 98, 101, 113
"C03C90513786"; % 1DB8, 80mm, 84, 86
"C9BE6C39667F"; % 1337, 79mm,81,72
"CA0E78A5FBB7"; % DA36, 80mm,82,89
"CDBFFDF40783"; % D82D, 108mm, 72, 65, 65, 61, 52
"E1D34659A004"; % DA05, 101mm,96,94
"E53B68E759EC"; % 0418, 110mm, 103, 116
"E10FBE416497"; % 033B, 99mm, 41, 59, 62, 61
"C52511681998"; % 15ED, 41mm, 39, 33, 49
"F01610672555"; % 43E6, 113mm, 108
"EAB9C5AC6589"; % 48C4, 92mm, 134, 141, 128
"EB1248ABAB99"; % 2FFD, 29mm, 49, 50, 34
"FDDA34A3BF9F"; % 0D40, 117mm, 118mm
"FC038EB37146"; % 48D3, 121mm, 115, 92
"F0C32D3DA717"; % 43A9, 83mm, 81, 83
"DCE3EE572F6F"; % 152F, 28mm, 20, 22
"FC5C101BF275"; % 2FEB, 40mm, 39, 33



% Specify the exact (tape) measurement distances between each device
% Notice that it is a symmetric matrix because the distance between 
% device i and device j are the same with the distance between 
% device j and device i. (in mm)
tape_mat = [NaN 2000 2854 2000
            2000 NaN 2004 2804
            2854 2000 NaN 2000
            2000 2804 2000 NaN]

N = 4 % Number of devices

% Memory allocation for the average distance matrix
dist_mat = zeros(N);

IDs = []; % Variable definition to store the IDs, will be updated inside the for loop below.

for i = 1:num_readings
    
    if i == 1
        [ID_1, dists_1] = read_from_anchor(device_name_1, true);
    else
        [ID_1, dists_1] = read_from_anchor(device_name_1, true);
    end

    [ID_2, dists_2] = read_from_anchor(device_name_2, false);
    [ID_3, dists_3] = read_from_anchor(device_name_3, false);
    [ID_4, dists_4] = read_from_anchor(device_name_4, false);

    % Print the number of readings per anchor for debug, each should be N-1
    assert(size(dists_1,1) == N-1)
    assert(size(dists_2,1) == N-1)
    assert(size(dists_3,1) == N-1)
    assert(size(dists_4,1) == N-1)
    
    % Make the distance matrix
    % Each row is data from one UWB anchor
    IDs = [ID_1 ID_2 ID_3 ID_4];
    
    dists = {dists_1, dists_2, dists_3, dists_4};

    dist_mat_current = zeros(N);
   
    for row = 1:N
        for col = 1:N
            index = dists{row}.Node_ID == IDs(col);
            if any(index)
                dist_mat_current(row, col) = dists{row}{index, 2};
            end
        end
    end

    % Print current reading
    dist_mat_current 

    % Add id to the average reading
    dist_mat = dist_mat + dist_mat_current/num_readings;
    
end

% Print the mean of all samples as the distance matrix
dist_mat

% Print the error compared to the ground truth in centimeters
e_cm = (tape_mat - dist_mat)/10
(e_cm + e_cm')/2


% % Now start the calibration

% Make the dist_matrix symmetric 
% Also print the symmetric version of the distance matrix
dist_mat = (dist_mat + dist_mat')/2

% Use nonlinear leastsquares to find the tag offsets
[offsets, delta, resid] = antenna_cal_min(tape_mat, dist_mat)

% Print the maximum residual value
max(abs(resid(:)))

% Obtain the hexadecimal values of the ids
dec2hex(IDs)

% Print the corresponding offsets these ids (in mm).
vpa(round(offsets))

% Note that, these offsets needs to be ADDED to the raw readings to obtain
% the accurate reading (not to be subtracted from them!)






%%
function [node_ID, dists] = read_from_anchor(device_name, is_initiatior)
    data_available = false;
    
    b = ble(device_name);
    
    device_info_c = characteristic(b, ...
        "680C21D9-C946-4C1F-9C11-BAA1C21329E7",...
        "1E63B1EB-D4ED-444E-AF54-C1E965192501");
    device_info_encoded = read(device_info_c, 'latest');
    disp('Got Node ID!')
    node_ID = typecast(uint8(device_info_encoded(1:2)), 'uint16');
    
    location_data_mode_c = characteristic(b, ...
        "680C21D9-C946-4C1F-9C11-BAA1C21329E7",...
        "A02B947E-DF97-4516-996A-1882521E0EAD");
    
    % Set to distances mode
    write(location_data_mode_c, uint8(1), "uint8");
    
    location_data_c = characteristic(b, ...
        "680C21D9-C946-4C1F-9C11-BAA1C21329E7",...
        "003BBDF2-C634-4B3D-AB56-7EC889B89A37");
    
    if is_initiatior
        
        subscribe(location_data_c) % Start automatic distance measurements
        location_data_c.DataAvailableFcn = @set_available;
        disp('Waiting for data...')
        while ~data_available
            pause(1)
        end
        dist_data_encoded = read(location_data_c, 'latest');
        disp('Got data!')
        unsubscribe(location_data_c)
        
    else
        % Don't need to subscribe, just read
        disp('Waiting for data...')
        dist_data_encoded = read(location_data_c, 'latest');
        disp('Got data!')
    end
    
    dists = decode_dist_data(dist_data_encoded(2:end));
    
    clear b
    
        function set_available(~,~)
            data_available = true;
        end
end

function dists = decode_dist_data(encoded)
    % First byte is distance count
    N = encoded(1);
    
    dists = table('Size', [N, 3], ...
        'VariableTypes', ["double", "double", "double"], ...
        'VariableNames',{'Node_ID', 'dist', 'quality'});
    
        for i = 1:N
            os = (i-1)*7;
            dists{i,1} = typecast(uint8(encoded(os + 1 + (1:2))), 'uint16');
            dists{i,2} = typecast(uint8(encoded(os + 1 + (3:6))), 'uint32');
            dists{i,3} = encoded(os + 1 + 7);
        end
end

function [offset, delta, resid] = antenna_cal_min(D_tape, D_uwb)

    options = optimoptions('lsqnonlin', "Display","iter-detailed", "FunctionTolerance",1e-12, "StepTolerance",1e-12, "MaxFunctionEvaluations",1e5);
    %options = optimoptions('lsqnonlin', "Display","iter-detailed", "MaxFunctionEvaluations",1000);
    offset_guess = zeros(4, 1);
    offset = lsqnonlin(@(offset)calibration_error(offset,  D_tape, D_uwb), offset_guess, [], [], options);
    delta = delta_D(offset);
    resid = D_tape - (D_uwb + delta_D(offset));
    
    function error = calibration_error(offset, D_tape, D_uwb)
        error = D_tape - (D_uwb + delta_D(offset));
        error(isnan(error)) = 0;
    end
end

function d = delta_D(offset)
    d = [0                 offset(1)+offset(2) offset(1)+offset(3) offset(1)+offset(4)
         offset(2)+offset(1)                 0 offset(2)+offset(3) offset(2)+offset(4)
         offset(3)+offset(1) offset(3)+offset(2)                 0 offset(3)+offset(4)
         offset(4)+offset(1) offset(4)+offset(2) offset(4)+offset(3)                0];
end