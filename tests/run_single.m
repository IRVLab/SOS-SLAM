clear

seq = 'mh1';
gt_file = strcat(strcat('gt/',seq),'.csv');
vo_file = strcat(strcat('results/SplineVIO/',seq),'.txt');

[gt, vo] = process(gt_file, vo_file, false);

rmse = sqrt(mean((vo(:) - gt(:)).^2));
disp(rmse)

figure
plot3(gt(:,1), gt(:,2), gt(:,3), 'g*'); 
hold on
plot3(vo(:,1), vo(:,2), vo(:,3), 'r*');
legend('ground-truth', 'SplineVIO')
axis equal