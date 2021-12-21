function [gt, vo] = process(gt_file, vo_file, sim3)
if ~isfile(vo_file)
    gt = [];
    vo = [];
    return
end

gt_raw = csvread(gt_file);
gt_raw = gt_raw(:,1:4);
gt_raw(:,1) = gt_raw(:,1) / 1e9;
vo_raw = load(vo_file);
vo_raw = vo_raw(:,1:4);

% if((vo_raw(1, 1) - vo_raw(end, 1)) / (gt_raw(1, 1) - gt_raw(end, 1)) < 0.8)
%     gt = [];
%     vo = [];
%     return
% end

rsi = 1; 
while(vo_raw(rsi,1)<gt_raw(1,1))
    rsi = rsi+1;
end
rei = size(vo_raw,1); 
while(vo_raw(rei,1)>gt_raw(end,1))
    rei = rei-1;
end
% if (vo_raw(rei, 1)-vo_raw(rsi, 1)) / (gt_raw(end,1)-gt_raw(1,1)) < 0.5
%     gt = [];
%     vo = [];
%     return
% end
vo = vo_raw(rsi:rei, :);

gt = zeros(size(vo));
ri=1;
gi=2;
while(gi<=size(gt_raw,1))
    if(ri>size(vo,1))
        break;
    end
    if(gt_raw(gi,1)<vo(ri,1)) 
        gi = gi+1;
        continue;
    end
    
    assert(gt_raw(gi-1,1)<=vo(ri,1) && vo(ri,1)<=gt_raw(gi,1));
    f = (vo(ri,1)-gt_raw(gi-1,1))/(gt_raw(gi,1)-gt_raw(gi-1,1));
    g_p = (1-f)*gt_raw(gi-1,2:4) + f*gt_raw(gi,2:4);
    gt(ri,1) = vo(ri,1);
    gt(ri,2:4) = g_p;
    ri = ri+1;
end

gt = gt(:,2:4);
vo = vo(:,2:4);

[reg,~,~]=absor(vo',gt','doScale',sim3);
for i=1:size(vo,1)
    vo(i,:) = reg.s * reg.R * vo(i,:)' + reg.t;
end
end

