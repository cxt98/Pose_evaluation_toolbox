function evaluate_poses_keyframe

    opt = globals();

    % read class names
    fid = fopen(opt.classfile, 'r');
    C = textscan(fid, '%s');
    object_names = C{1};
    fclose(fid);
    id_transform = [eye(3), [0;0;0]];

    % load model points
    num_objects = numel(object_names);
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
        models{i} = obj.v';
    end

    % save results
    distances_sys = zeros(100000, length(opt.labels));
    distances_non = zeros(100000, length(opt.labels));
    errors_rotation = zeros(100000, length(opt.labels)); 
    errors_translation = zeros(100000, length(opt.labels));
    results_frame_id = zeros(100000, length(opt.labels));
    results_object_id = zeros(100000, length(opt.labels));
    results_cls_id = zeros(100000, length(opt.labels));
    
    
    for m = 1: length(opt.result_path)
        count = 0;
        result_path = opt.result_path{m};
        for i = 1: length(dir(result_path))-2
            frame_id = i - 1;

            % load both gt and estimate in the same file
            filename = sprintf('%s/%04d.mat', result_path, frame_id);
            all_output = load(filename);

            % for each pose pair
            for j = 1:numel(all_output.cls_indexes)
                cls_index = all_output.cls_indexes(j);

                count = count + 1;
                fprintf('\t%d\n', count);

                results_frame_id(count, m) = frame_id;
                results_object_id(count, m) = j;
                results_cls_id(count, m) = cls_index;

                if opt.switch_cleargrasp0toolset1
                    % toolset-generalized-affordance format: both gt and
                    % estimation saved as [x, y, z, qw, qx, qy, qz]
                    RT_gt(1:3, 1:3) = quat2rotm(all_output.gt_poses(j, 4:7));
                    RT_gt(:, 4) = all_output.gt_poses(j, 1:3);
                    RT(1:3, 1:3) = quat2rotm(all_output.poses(j, 4:7));
                    RT(:, 4) = all_output.poses(j, 1:3);
                else
                    % cleargrasp-densefusion format: gt saved in 3*4 matrix,
                    % estimation saved as [qw, qx, qy, qz, x, y, z]
                    RT_gt = all_output.gt_poses(:, :, j);
                    RT(1:3, 1:3) = quat2rotm(all_output.poses(j, 1:4));
                    RT(:, 4) = all_output.poses(j, 5:7);
                end
                if ~isequal(RT, id_transform)
                    distances_sys(count, m) = adi(RT, RT_gt, models{cls_index}');
                    distances_non(count, m) = add(RT, RT_gt, models{cls_index}');
                    errors_rotation(count, m) = re(RT(1:3, 1:3), RT_gt(1:3, 1:3));
                    errors_translation(count, m) = te(RT(:, 4), RT_gt(:, 4));
                else
                    distances_sys(count, m) = inf;
                    distances_non(count, m) = inf;
                    errors_rotation(count, m) = 180;
                    errors_translation(count, m) = inf;
                end
            end
        end
    end
    distances_sys = distances_sys(1:count, :);
    distances_non = distances_non(1:count, :);
    errors_rotation = errors_rotation(1:count, :);
    errors_translation = errors_translation(1:count, :);
    results_frame_id = results_frame_id(1:count, :);
    results_object_id = results_object_id(1:count, :);
    results_cls_id = results_cls_id(1:count, :);
    save(opt.output_file, 'distances_sys', 'distances_non',...
        'errors_rotation', 'errors_translation', 'results_frame_id', 'results_object_id', 'results_cls_id');
end

function pts_new = transform_pts_Rt(pts, RT)
    %     """
    %     Applies a rigid transformation to 3D points.
    % 
    %     :param pts: nx3 ndarray with 3D points.
    %     :param R: 3x3 rotation matrix.
    %     :param t: 3x1 translation vector.
    %     :return: nx3 ndarray with transformed 3D points.
    %     """
    n = size(pts, 2);
    pts_new = RT * [pts; ones(1, n)];
end

function error = add(RT_est, RT_gt, pts)
    %     """
    %     Average Distance of Model Points for objects with no indistinguishable views
    %     - by Hinterstoisser et al. (ACCV 2012).
    % 
    %     :param R_est, t_est: Estimated pose (3x3 rot. matrix and 3x1 trans. vector).
    %     :param R_gt, t_gt: GT pose (3x3 rot. matrix and 3x1 trans. vector).
    %     :param model: Object model given by a dictionary where item 'pts'
    %     is nx3 ndarray with 3D model points.
    %     :return: Error of pose_est w.r.t. pose_gt.
    %     """
    pts_est = transform_pts_Rt(pts, RT_est);
    pts_gt = transform_pts_Rt(pts, RT_gt);
    diff = pts_est - pts_gt;
    error = mean(sqrt(sum(diff.^2, 1)));
end

function error = adi(RT_est, RT_gt, pts)
    %     """
    %     Average Distance of Model Points for objects with indistinguishable views
    %     - by Hinterstoisser et al. (ACCV 2012).
    % 
    %     :param R_est, t_est: Estimated pose (3x3 rot. matrix and 3x1 trans. vector).
    %     :param R_gt, t_gt: GT pose (3x3 rot. matrix and 3x1 trans. vector).
    %     :param model: Object model given by a dictionary where item 'pts'
    %     is nx3 ndarray with 3D model points.
    %     :return: Error of pose_est w.r.t. pose_gt.
    %     """
    pts_est = transform_pts_Rt(pts, RT_est);
    pts_gt = transform_pts_Rt(pts, RT_gt);

    % Calculate distances to the nearest neighbors from pts_gt to pts_est
    MdlKDT = KDTreeSearcher(pts_est');
    [~, D] = knnsearch(MdlKDT, pts_gt');
    error = mean(D);
end

function error = re(R_est, R_gt)
    %     """
    %     Rotational Error.
    % 
    %     :param R_est: Rotational element of the estimated pose (3x1 vector).
    %     :param R_gt: Rotational element of the ground truth pose (3x1 vector).
    %     :return: Error of t_est w.r.t. t_gt.
    %     """

    error_cos = 0.5 * (trace(R_est * inv(R_gt)) - 1.0);
    error_cos = min(1.0, max(-1.0, error_cos));
    error = acos(error_cos);
    error = 180.0 * error / pi;
end

function error = te(t_est, t_gt)
    % """
    % Translational Error.
    % 
    % :param t_est: Translation element of the estimated pose (3x1 vector).
    % :param t_gt: Translation element of the ground truth pose (3x1 vector).
    % :return: Error of t_est w.r.t. t_gt.
    % """
    error = norm(t_gt - t_est);
end