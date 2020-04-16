function plot_accuracy

    opt = globals();
    
    color = {'r', 'y', 'g', 'b', 'm'};
%     leng = {'iterative', 'PoseCNN+ICP', 'per-pixel', '3DCoordinate', ...
%         '3D'};
    aps = zeros(length(color), 1);
    lengs = cell(length(opt.labels), 1);
    close all;

    % load results
    load(opt.output_file);

    % read class names
    fid = fopen(opt.classfile, 'r');
    C = textscan(fid, '%s');
    classes = C{1};
    classes{end+1} = 'All objects';
    fclose(fid);
    
    font_size = opt.font_size;
    max_distance = 0.1;
    metrics = {@(x, y) distances_sys(x, y), @(x, y) distances_non(x, y),...
                @(x, y) errors_rotation(x, y), @(x, y) errors_translation(x, y)};
    xlabels = {'Average distance threshold in meter (symmetry)',...
        'Average distance threshold in meter (non-symmetry)',...
        'Rotation angle threshold',...
        'Translation threshold in meter'};
    if ~exist(opt.output_plotpath, 'dir')
        mkdir(opt.output_plotpath);
    end
    
    
    hf = figure('units', 'normalized', 'outerposition', [0 0 1 1]);
    % for each class
    for k = 1:numel(classes)
        fprintf('%s\n', classes{k});
        for j = 1: 4
            % distance symmetry, distance non-symmetry, translation,
            % rotation
            subplot(2, 2, j);
            hold on
            for i = 1: length(opt.labels)
                h = xlabel(xlabels{j});
                set(h, 'FontSize', font_size);
                h = ylabel('accuracy');
                set(h, 'FontSize', font_size);
                h = title(classes{k}, 'Interpreter', 'none');
                set(h, 'FontSize', font_size);
                xt = get(gca, 'XTick');
                set(gca, 'FontSize', font_size)
                index = find(results_cls_id(:, i) == k);
                if isempty(index)
                    if j <= 2 && k ~= numel(classes)
                        lengs{i} = sprintf('%s(AUC:%.2f)(<2cm:%.2f)', opt.labels{i}, 0, 0);
                        continue;
                    else
                        index = 1:size(distances_sys, 1);
                    end
                end
                D = metrics{j}(index, i);
                if j ~= 3
                    D(D > max_distance) = inf;
                else  % rotation
                    xlim([0 180])
                end
                d = sort(D);
                n = numel(d);
                c = numel(d(d < 0.02));
                accuracy = cumsum(ones(1, n)) / n;

                plot(d, accuracy, color{i}, 'LineWidth', 4);
                if j <= 2 % only show for add, adds
                    aps(i) = VOCap(d, accuracy);
                    lengs{i} = sprintf('%s(AUC:%.2f)(<2cm:%.2f)', opt.labels{i}, aps(i)*100, (c/n)*100);
                end
                
            end
            h = legend(lengs, 'Location', 'southeast');
            set(h, 'FontSize', font_size);
            hold off
        end
        filename = sprintf('%s/%s.png', opt.output_plotpath, classes{k});
        hgexport(hf, filename, hgexport('factorystyle'), 'Format', 'png');
        clf;
    end
end

function ap = VOCap(rec, prec)

    index = isfinite(rec);
    if ~any(index)
        ap = 0;
        return;
    end
    rec = rec(index);
    prec = prec(index)';

    mrec=[0 ; rec ; 0.1];

    mpre=[0 ; prec ; prec(end)];
    for i = 2:numel(mpre)
        mpre(i) = max(mpre(i), mpre(i-1));
    end
    i = find(mrec(2:end) ~= mrec(1:end-1)) + 1;
    ap = sum((mrec(i) - mrec(i-1)) .* mpre(i)) * 10;
end