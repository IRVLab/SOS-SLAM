close all
clear

types = ["SOS-SLAM"];
 
% 0.066 0.056 0.142 0.131 0.129
% 0.087 × × 0.103 0.111 ×
% 0.085 0.186 0.114 0.142 × 0.137
tests = ["mh1","mh2","mh3","mh4","mh5", ...
    "v11", "v12", "v13", "v21", "v22", "v23"];

rmses = getRMSE(types, tests);
showRMSE(types, tests, rmses, ...
    [0.5 3 5.5 7 8.5 10 11.5, 14.5, 17.5], ... 
    {'|', 'MH', '|', 'V1', '|', 'V2', '|', 'TR', '|'});


function rmses = getRMSE(types, tests)
runs = 10;
rmses = zeros(length(types),length(tests),runs);
for t=1:length(types)
    vo_dir = strcat(strcat('results/', types(t)), '/');
    for i=1:length(tests)
        for r=1:runs
            test_run = strcat(strcat(tests(i), '_'), int2str(r));
            gt_file = strcat(strcat('gt/', tests(i)), '.csv');
            vo_file = strcat(strcat(vo_dir, test_run), '.txt');
            
            [gt, vo] = process(gt_file, vo_file, types(t)=='DSO');
            
            if isempty(gt)
                rmses(t,i,r) = 999;
            else
                rmses(t,i,r) = sqrt(mean((vo(:) - gt(:)).^2));
            end
        end
    end
end
end

function showRMSE(types, tests, rmses, x_tic, x_lbl)
disp(tests)
%     disp(rmses);
for t=1:length(types)
    disp(types(t))
    rmse = squeeze(rmses(t,:,:))';
    m_rmse = median(rmse);
    m_rmse(m_rmse>10)=-1;
    disp(m_rmse)
    subplot(length(types), 1, t);
    fg = gca;
    imagesc(fg, rmse, [0,0.5])
    xticks(x_tic)
    xticklabels(x_lbl)
    yticklabels([])
    colormap(fg, jet(256))
    colorbar
    axis image
    title(types(t), 'Interpreter', 'none')
end
end