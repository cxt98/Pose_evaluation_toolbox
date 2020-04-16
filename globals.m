function opt = globals()
    opt.switch_cleargrasp0toolset1 = 0;
    if ~opt.switch_cleargrasp0toolset1
    %% cleargrasp synthetic dataset
        opt.root = '/home/cxt/Documents/research/lf_perception/598-007-project';
        opt.classfile = strcat(opt.root, '/classes.txt');
        opt.num_classes = 5;
        opt.labels = {'Densefusion'};
        opt.model_path = strcat(opt.root, '/cleargrasp-3d-models-fixed/');
%         opt.result_path = strcat(opt.root, '/DenseFusion/experiments/eval_result/cleargrasp/Densefusion_wo_refine_result/');
        opt.result_path = {strcat(opt.root, '/DenseFusion/experiments/eval_result/cleargrasp/Densefusion_wo_refine_result/')};
        opt.output_file = 'results_cleargrasp.mat';
        opt.output_plotpath = 'plots_cleargrasp';
        opt.output_imagepath = {'images_cleargrasp'};
        opt.testlist_file = strcat(opt.root, '/DenseFusion/datasets/cleargrasp/dataset_config/test_data_cup_list.txt');
        opt.test_imagepath = strcat(opt.root, '/cleargrasp-testing-validation/synthetic-val/');
        opt.intrinsic_matrix_color = [1386.4, 0, 960; 0, 1386.4 540; 0, 0 ,1]; % for 1920*1080 image
%         opt.intrinsic_matrix_color = [739.4, 0, 512; 0, 739.4 288; 0, 0 ,1]; % for 1024*576 image
    else
    %% affordance toolset dataset
        opt.root = '/home/cxt/Documents/research/affordance/tool_dataset/tool_parts';
        opt.classfile = strcat(opt.root, '/classes.txt');
        opt.num_classes = 18;
        opt.model_path = strcat(opt.root, '/mesh_models/all_obj');
        opt.labels = {'RGBonly', 'RGBDfull'};
        opt.result_path = {'/home/cxt/Documents/research/affordance/generalized-affordances/testing_output/toolset/RGBD_w_eval/result_eval/matched_poses_mat',...
            '/home/cxt/Documents/research/affordance/generalized-affordances/testing_output/toolset/RGB_w_eval/result_eval/matched_poses_mat'};
        opt.output_file = 'results_toolset.mat';
        opt.output_plotpath = 'plots_toolset';
        opt.output_imagepath = {};
        for i = 1: length(opt.labels)
            opt.output_imagepath{i} = strcat('images_toolset_', opt.labels{i});
        end
        opt.testlist_file = strcat(opt.root, '/test_images.txt');
        opt.intrinsic_matrix_color = [536.5441342081624, 0, 324.1496087846382; 0, 537.6663041098749 224.2990274169881; 0, 0 ,1];
        assert(length(opt.labels) == length(opt.result_path));
    end
    opt.font_size = 36;
    opt.class_colors = uint8([255, 255, 255; 255, 0, 0; 0, 255, 0; 0, 0, 255; 255, 255, 0; 255, 0, 255; 0, 255, 255;
                      128, 0, 0; 0, 128, 0; 0, 0, 128; 128, 128, 0; 128, 0, 128; 0, 128, 128;
                      64, 0, 0; 0, 64, 0; 0, 0, 64; 64, 64, 0; 64, 0, 64; 0, 64, 64; 
                      192, 0, 0; 0, 192, 0; 0, 0, 192]);

    %% unused below

%     opt.seq_num = 92;
%     opt.nums = [762, 1112, 1719, 2299, 2172, 1506, 1626, 2018, 2991, 1591, 1898, ...
%         1107, 1104, 1800, 1619, 2305, 1335, 1716, 1424, 2218, 1873, 731, 1153, 1618, ...
%         1401, 1444, 1607, 1448, 1558, 1164, 1124, 1952, 1254, 1567, 1554, 1668, ...
%         2512, 2157, 3467, 3160, 2393, 2450, 2122, 2591, 2542, 2509, 2033, 2089, ...
%         2244, 2402, 1917, 2009, 900, 837, 1929, 1830, 1226, 1872, 1720, 1864, ...
%         754, 533, 680, 667, 668, 653, 801, 849, 884, 784, 1016, 951, 890, 719, 908, ...
%         694, 864, 779, 689, 789, 788, 985, 743, 953, 986, 890, 897, 948, 453, 868, 842, 890] - 1;
% 
%     opt.width = 640;
%     opt.height = 480;
% 
%     opt.intrinsic_matrix_color = [1066.778, 0, 312.9869; 0, 1067.487 241.3109; 0, 0 ,1];
%     opt.intrinsic_matrix_depth = [567.6188, 0, 310.0724; 0, 568.1618 242.7912; 0, 0 ,1];
% 
%     opt.depth2color = [ 0.9997563, 0.02131301, -0.005761033, 0.02627148; ...
%                         -0.02132165, 0.9997716, -0.001442874, -0.0001685539; ...
%                         0.005728965, 0.001565357, 0.9999824, 0.0002760285  ];
% 
%     opt.intrinsic_matrix_color_cmu = [1077.836, 0, 323.7872; 0, 1078.189 279.6921; 0, 0 ,1];
%     opt.intrinsic_matrix_depth_cmu = [576.3624, 0, 319.2682; 0, 576.7067 243.8256; 0, 0 ,1];
% 
%     opt.depth2color_cmu = [ 0.9999962, -0.002468782, 0.001219765, 0.02640966; ...
%        0.002466791, 0.9999956, 0.001631345, -9.9086e-05; ...
%        -0.001223787, -0.00162833, 0.9999979, 0.0002972445];
% 
end                  