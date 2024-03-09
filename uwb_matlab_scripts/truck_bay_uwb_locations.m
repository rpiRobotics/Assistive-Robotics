% Ordering
% 033B
% D82D
% 43A9
% 2FFD
% anchor_names = ["DW033B";"DWD8D2";"Dw43A9";"Dw2FFD"]
% Anchor addresses, use blelist command to figure them out
anchor_names = ["E10FBE416497"; "CDBFFDF40783"; "F0C32D3DA717"; "EB1248ABAB99"]

% heights of each UWB anchor in m
heights = [3.120; 3.310; 3.308; 3.268]

% Distance matrix (total xyz, NOT projected to xy)
dist_matrix = [   NaN      0     0     0 
               8.268    NaN     0     0
               10.375  3.669  NaN     0
                4.309  8.930 9.436   NaN];

% ALSO SEE BELOW TO INCLUDE CUSTOM WORLD ORIGIN ADJUSTMENTS

dist_matrix = dist_matrix + dist_matrix'


init_guess = zeros([2 4]);
init_guess(1,2) = 1;
init_guess(:,3) = 1;
init_guess(2,4:end) = 1;

[locations, min_dist_mat] = solve_locations_with_z(dist_matrix, heights, init_guess)

res = (dist_matrix - min_dist_mat)*1e2
max(abs(res(:)))

locations_z = [locations; heights']

vecnorm(locations_z(:,1) - locations_z(:,2))

%%
figure;
plot3(locations_z(1,:), locations_z(2,:), locations_z(3,:), '-xr')
axis equal
grid on
%%
%%% CUSTOM WORLD ORIGIN ADJUSTMENTS %%%
% For correcting the orientation, use a known angle
alpha = asind(1.290/8.2679) % deg
R = rotz(alpha) % rotation matrix

% Apply rotation matrix to the locations
locations_z_rotated = R*locations_z

% Apply position correction for the first anchor position wrt fence
locations_z_rotated = locations_z_rotated + [0.55;0.1;0]

% Apply anothert position correction for our definition of world origin
locations_z_rotated = locations_z_rotated + [-5.0;-3.0;0]

% locations_z_rotated = 3×4    
%    -4.4500    3.7166    4.7351   -4.6885
%    -2.9000   -1.6100    1.9158    1.4006
%     3.1200    3.3100    3.3080    3.2680

%%
figure;
plot3(locations_z_rotated(1,:), locations_z_rotated(2,:), locations_z_rotated(3,:), '-xb')
axis equal
grid on
%%
% Write the positions to the anchors
% loc_mm = 1e3*locations_z;
loc_mm = 1e3*locations_z_rotated;


% WARNING: THERE IS A BUG IN THIS FUNCTION WHEN WRITING NEGATIVE VALUES,
% TODO: FIX IT
% WORKAROUND: MANUALLY ENTER CALCULATED POSITIONS FROM THE DRTLS ANDROID APP
write_to_anchor(anchor_names(1), loc_mm(1,1), loc_mm(2,1), loc_mm(3,1));
write_to_anchor(anchor_names(2), loc_mm(1,2), loc_mm(2,2), loc_mm(3,2));
write_to_anchor(anchor_names(3), loc_mm(1,3), loc_mm(2,3), loc_mm(3,3));
write_to_anchor(anchor_names(4), loc_mm(1,4), loc_mm(2,4), loc_mm(3,4));

%%
function write_to_anchor(device_name, x, y, z)
disp('connecting...')
b = ble(device_name);

position_c = characteristic(b, ...
    "680C21D9-C946-4C1F-9C11-BAA1C21329E7", ...
    "F0F26C9B-2C8C-49AC-AB60-FE03DEF1B40C");
% Each LSB is 1 mm
q = 100;
d = [ typecast(uint32(x), 'uint8') typecast(uint32(y), 'uint8') typecast(uint32(z), 'uint8') uint8(q)];
write(position_c, d, "uint8")

clear b
disp('Wrote data!')
end