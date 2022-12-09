% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clean env
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear;
clc;
cvx_clear;


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system setup
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
system_setup;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% system solve
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_acs;

solve_acs_cantelli;


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh = figure();
fh.WindowState = 'maximized';
subplot(7,4,[1:4]);
hold on
p1=plot(nan, nan, '-or');
p2=plot(nan, nan, '-sk');
p3=patch(nan, nan, 'b', 'FaceAlpha', 0.1);
p4=patch(nan, nan, 'g', 'FaceAlpha', 0.1);
legend([p1,p2,p3,p4], ['Proposed Method', "Cantelli's Inequality", 'Line of Sight', 'Target Set'], ...
    'Orientation','horizontal', ...
    'Location', 'south', ...
    'NumColumns', 4);
axis([0 0.1 0 0.1]);
axis off
hold off

subplot(7,4,[5,6,9,10,13,14,17,18,21,22,25,26]);
hold on
plot(traj_holder(1:6:end, iter),traj_holder(2:6:end, iter), '-or','MarkerSize',15);
plot(traj_holder_cantelli(1:6:end, iterc),traj_holder_cantelli(2:6:end, iterc), '-sk','MarkerSize',15);
patch([0; 10; 10], [0; -5; 5], 'b', 'FaceAlpha', 0.1);
patch([0; 2; 2; 0], [-0.5; -0.5; 0.5; 0.5], 'g', 'FaceAlpha', 0.1);
xlabel('x');
ylabel('y');
axis([0 12 -6 6]);
hold off

subplot(7,4,[7,8,11,12,15,16,19,20,23,24.27,28]);
hold on
plot(traj_holder(1:6:end, iter), traj_holder(3:6:end, iter), '-or','MarkerSize',15);
plot(traj_holder_cantelli(1:6:end, iterc), traj_holder_cantelli(3:6:end, iterc), '-sk','MarkerSize',15);
patch([0; 10; 10], [0; -5; 5], 'b', 'FaceAlpha', 0.1);
patch([0; 2; 2; 0], [-0.5; -0.5; 0.5; 0.5], 'g', 'FaceAlpha', 0.1);
xlabel('x');
ylabel('z');
axis([0 12 -6 6]);
hold off
