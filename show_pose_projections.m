% seq_id: 0 ~ 91
% The *-meta.mat file in the YCB-Video dataset contains the following fields:
% center: 2D location of the projection of the 3D model origin in the image
% cls_indexes: class labels of the objects
% factor_depth: divde the depth image by this factor to get the actual depth vaule
% poses: 6D poses of objects in the image
% intrinsic_matrix: camera intrinsics
% rotation_translation_matrix: RT of the camera motion in 3D
% vertmap: coordinates in the 3D model space of each pixel in the image
function show_pose_annotations

    opt = globals();

    % read class names
    fid = fopen(opt.classfile, 'r');
    C = textscan(fid, '%s');
    object_names = C{1};
    fclose(fid);
    num_objects = numel(object_names);

    % load CAD models
    disp('loading 3D models...');
    models = cell(num_objects, 1);
%     end
    for i = 1:num_objects
        matfilename = sprintf('models/%s.mat', object_names{i});
        if exist(matfilename, 'file')
            object = load(matfilename);
            obj = object.obj;
        else
            filename = sprintf('%s/%s.obj', opt.model_path, object_names{i});
            obj = load_obj_file(filename);
            save(matfilename, 'obj');
            disp(filename);
        end
        models{i} = obj;
    end

    close all;
    fid = fopen(opt.testlist_file, 'r');
    C = textscan(fid, '%s\n');
    test_list = C{1};
    fclose(fid);
    
    hf = figure('units', 'normalized', 'outerposition', [0 0 1 1]);
    
    % for each method
    for i = 1: length(opt.labels)
        if ~exist(opt.output_imagepath{i}, 'dir')
            mkdir(opt.output_imagepath{i});
        end
        % for each frame
        for k = 1: length(test_list)
            % read image
            if opt.switch_cleargrasp0toolset1 == 0
                tmp = split(test_list{k},'/');
                folder = tmp{1}; imgind = tmp{2};
                imfilename = sprintf('%s/%s/rgb-imgs/%s-rgb.jpg', opt.test_imagepath, folder, imgind);
                I = imread(imfilename);
            else
                imfilename = test_list{k};
                I = imread(imfilename);
            end
            metafilename = sprintf('%s/%04d.mat', opt.result_path{i}, k-1);
            imsavename = split(imfilename,'/');
            imsavename = imsavename{end};
            object = load(metafilename);


            % copied from evaluate_poses_keyframe.m
            if opt.switch_cleargrasp0toolset1
                % toolset-generalized-affordance format: both gt and
                % estimation saved as [x, y, z, qw, qx, qy, qz]
                RT_gt = zeros(3, 4, size(object.gt_poses, 1));
                RT = RT_gt;
                for j = 1: size(object.gt_poses, 1)
                    RT_gt(1:3, 1:3, j) = quat2rotm(object.gt_poses(j, 4:7));
                    RT_gt(:, 4, j) = object.gt_poses(j, 1:3);
                    RT(1:3, 1:3, j) = quat2rotm(object.poses(j, 4:7));
                    RT(:, 4, j) = object.poses(j, 1:3);
                end
%                 object.cls_indexes = object.cls_indexes;
                for t = 1: length(object.cls_indexes)
                    if object.cls_indexes(t) > 4
                        object.cls_indexes(t) = object.cls_indexes(t) + 1;
                    end
                end
                
            else
                % cleargrasp-densefusion format: gt saved in 3*4 matrix,
                % estimation saved as [qw, qx, qy, qz, x, y, z]
                RT_gt = object.gt_poses;
                RT = RT_gt;
                for j = 1: size(object.gt_poses, 3)
                    RT(1:3, 1:3, j) = quat2rotm(object.poses(j, 1:4));
                    RT(:, 4, j) = object.poses(j, 5:7);
                end
            end

            subplot(1, 2, 1);
            imshow(I);
            project2d(RT_gt, object.cls_indexes, models, opt, I);
            h = title('Ground truth pose projection', 'Interpreter', 'none');
            set(h, 'FontSize', opt.font_size);

            subplot(1, 2, 2);
            imshow(I);
            project2d(RT, object.cls_indexes, models, opt, I);
            h = title('Estimated pose projection');
            set(h, 'FontSize', opt.font_size);
    %         pause
            fprintf('%s\n', imsavename);
            hgexport(hf, fullfile(opt.output_imagepath{i}, imsavename), hgexport('factorystyle'), 'Format', 'png');
        end
    end
end

function project2d(poses, cls_indexes, models, opt, I)
    hold on;
    % sort objects according to distances
    num = numel(cls_indexes);
    id_transform = [eye(3), [0;0;0]];

    % for each object
    for ind = 1:num

        % load RT_o2c
        RT_o2c = poses(:,:,ind);
        if ~isequal(RT_o2c, id_transform)

            % projection
            x3d = models{cls_indexes(ind)}.v';
            x2d = project(x3d, opt.intrinsic_matrix_color, RT_o2c);

            % bounding boxes
            vmin = min(x2d, [], 1);
            vmax = max(x2d, [], 1);
            x1 = max(vmin(1), 0);
            y1 = max(vmin(2), 0);
            x2 = min(vmax(1), size(I,2));
            y2 = min(vmax(2), size(I,1));     

            % draw
            if opt.switch_cleargrasp0toolset1 == 0
                patch('vertices', x2d, 'faces', models{cls_indexes(ind)}.f3', ...
                    'FaceColor', opt.class_colors(cls_indexes(ind)+1,:), 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            else
                plot(x2d(:, 1), x2d(:, 2), '.', 'color', opt.class_colors(cls_indexes(ind)+1,:));
            end
            if x1 >= 0 && x2 <= size(I, 2) && x2 > x1 && y1 >= 0 && y2 <= size(I, 1) && y2 > y1
                rectangle('Position', [x1, y1, x2-x1, y2-y1], ...
                    'EdgeColor', opt.class_colors(cls_indexes(ind)+1,:), 'LineWidth', 2);    
            end
        end
    end
    hold off;
end

%% unused
function vertex_targets = generate_vertex_targets(im_label, cls_indexes, center, poses, num_classes)

    % sort objects according to distances
    num_objects = numel(cls_indexes);
    distances = zeros(num_objects, 1);
    for j = 1:num_objects
        distances(j) = poses(3, 4, j);
    end

    width = size(im_label, 2);
    height = size(im_label, 1);
    vx = -1.5 * ones(height, width);
    vy = -1.5 * ones(height, width);
    vz = -max(distances) * ones(height, width) - 0.1;

    c = zeros(1, 2);
    for i = 1:num_classes
        [y, x] = find(im_label == i);
        if ~isempty(x)
            ind = find(cls_indexes == i);
            c(1) = center(ind, 1);
            c(2) = center(ind, 2);
            z = poses(3, 4, ind);
            R = repmat(c, length(x), 1) - [x, y];
            % compute the norm
            N = sqrt(sum(R .^ 2, 2)) + 1e-10;
            % normalization
            R = R ./ [N N];
            % assignment
            indp = sub2ind(size(im_label), y, x);
            vx(indp) = R(:, 1);
            vy(indp) = R(:, 2);
            vz(indp) = -z;
        end
    end

    vertex_targets = zeros(height, width, 3);
    vertex_targets(:, :, 1) = rescale_image(vx);
    vertex_targets(:, :, 2) = rescale_image(vy);
    vertex_targets(:, :, 3) = vz;
end

function im = rescale_image(im)

    vmax = max(max(im));
    vmin = min(min(im));

    a = 1.0 / (vmax - vmin);
    b = -1.0 * vmin / (vmax - vmin);

    im = a * im + b;
end