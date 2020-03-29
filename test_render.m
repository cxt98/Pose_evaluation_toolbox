n = 1;

obj_name = "heart-bath-bomb";
% cup-with-waves: -z/2
% flower-bath-bomb: seems +y/4 with another rotation?
% heart-bath-bomb:
root_dir = "/home/cxt/Documents/research/lf_perception/598-007-project/";

obj_modelpath = strcat(root_dir, "cleargrasp-3d-models/", obj_name, "_downsampled.pcd");
objmodel = pcread(obj_modelpath);

obj_posefile = strcat(root_dir, "cleargrasp-dataset-train/", obj_name, "-train/meta-files/000000000.mat");
load(obj_posefile);

rgb_img = imread(strcat(root_dir, "/cleargrasp-dataset-train/", obj_name, "-train/rgb-imgs/000000000-rgb.jpg"));

x_axis_rads = 1.2112585306167603; y_axis_rads = 0.7428327202796936;
focal_length_longest_axis = 23.10693359375;
[img_height, img_width, ~] = size(rgb_img);
cx = img_width / 2; cy = img_height / 2; 
fx = cx / tan(x_axis_rads / 2); %/ focal_length_longest_axis; 
fy = cy / tan(y_axis_rads / 2); %/ focal_length_longest_axis;

camera_matrix = [[fx, 0, cx]; [0, fy, cy]; [0, 0, 1]];

bin_img = false(img_height, img_width);
for i = 1: size(poses, 3)
    pose = poses(:, :, i);
    tform = [pose; [0, 0, 0, 1]]';
    local_points = objmodel.Location;
    x_offset = 0; y_offset = 0; z_offset = 0;
%     x_offset = (max(local_points(:, 1)) - min(local_points(:, 1))) / 2;
%     y_offset = (max(local_points(:, 2)) - min(local_points(:, 2))) / 2;
%     z_offset = (max(local_points(:, 3)) - min(local_points(:, 3))) / 2;
    local_points(:, 3) = local_points(:, 3) - z_offset;
    points = local_points * pose(:, 1:3)' + pose(:, 4)';
    
    % own projecter
    x_z = points(:, 1) ./ points(:, 3);
    y_z = points(:, 2) ./ points(:, 3);
    u_v = camera_matrix * [x_z'; y_z'; ones(1, length(x_z))];
    u = round(u_v(1, :)); v = round(u_v(2, :));
    valid_idx = u >= 1 & u <= img_width & v >= 1 & v <= img_height;
    if nnz(valid_idx) > 0
        u = double(u(valid_idx)); v = double(v(valid_idx));
        u_v_unique = unique([u; v]', 'rows', 'stable');
        u = u_v_unique(:, 1); v = u_v_unique(:, 2);
        boundary_idx = convhull(v, u);
        [x, y] = meshgrid(min(u):max(u), min(v):max(v));
        [in, on] = inpolygon(x,y, u(boundary_idx), v(boundary_idx));
        bin_img(sub2ind(size(bin_img), y(in), x(in))) = 1;
    end
    
%     hold on, axis equal, xlim([1 img_width]), ylim([1 img_height])
%     plot(u, v, 'd');
end
figure
bin_img = fliplr(bin_img);
rgb_img(bin_img == 0) = 0;
imshow(rgb_img);