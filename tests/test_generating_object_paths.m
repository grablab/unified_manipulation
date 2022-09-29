
clear
clc
close all

[goal_xyz, goal_ang, env, taskname_add] = taskSmoothHelical();
% [goal_xyz, goal_ang, env, taskname_add] = taskSharpPath();
% [goal_xyz, goal_ang, env, taskname_add] = taskCupPour();
% [goal_xyz, goal_ang, env, taskname_add] = taskICRACollide();
% [goal_xyz, goal_ang, env, taskname_add] = taskSmallMovt();

close all

%% Task 1: Move cup over obstacle and pour

function [goal_xyz, goal_ang, env, taskname_add] = taskCupPour()

xyz_way = [0.05 -0.3 0.6;
           0.4 -0.3 0.6;
           0.1 -0.2 0.7
           0.0   0   0.8;
           0.1   0.1  0.9;
           0.3   0.25  1;
           0.4   0.25  0.8];

ang_way = [1e-4 pi/2 1e-4;
           1e-4 pi/2 1e-4;
           1e-4 pi/2 1e-4;
           1e-4 pi/2 1e-4;
           1e-4 pi/2 1e-4;
           1e-4 pi/2+pi/8 1e-4;
           1e-4 pi 1e-4];

interp_points= linspace(1,size(xyz_way, 1), 64);
xx = interp1(1:size(xyz_way, 1),xyz_way(:, 1),interp_points, "spline");
yy = interp1(1:size(xyz_way, 1),xyz_way(:, 2),interp_points, "spline");
zz = interp1(1:size(xyz_way, 1),xyz_way(:, 3),interp_points, "spline");
angx = interp1(1:size(xyz_way, 1),ang_way(:, 1),interp_points, "spline");
angy = interp1(1:size(xyz_way, 1),ang_way(:, 2),interp_points, "spline");
angz = interp1(1:size(xyz_way, 1),ang_way(:, 3),interp_points, "spline");


goal_xyz = [xx' yy' zz'];
goal_ang = [angx' angy' angz'];
    
env = makeCupPourWorld();

taskname_add = 'task - cup pour';

end

%% Task 2: Follow path with sharp turns while always facing away

function [goal_xyz, goal_ang, env, taskname_add] = taskSharpPath()

xyz_way = [0.4 -0.4 1; 
           0.4 -0.4 0.4;
           0.4   0   1;
           0.4 0.4 0.4;
           0   0.4  1;
           -0.4 0.4 1;
           -0.4 0.4 0.4;
           -0.4  0   1
           -0.4 -0.4 0.4;
           0    -0.4  1
           0.4 -0.4 1];

ang_way = [0 pi/2 0;
           0 pi/2 0;
           0 pi/2 0;
           0 pi/2 0;
           -pi/2 0 0;
           -pi/2 0 0;
           -pi/2 0 0;
           0 -pi/2 0;
           0 -pi/2 0;
           pi/2 0 0;
           pi/2 0 0;];

interp_points= linspace(1,size(xyz_way, 1), 84);
xx = interp1(1:size(xyz_way, 1),xyz_way(:, 1),interp_points);
yy = interp1(1:size(xyz_way, 1),xyz_way(:, 2),interp_points);
zz = interp1(1:size(xyz_way, 1),xyz_way(:, 3),interp_points);
angx = interp1(1:size(xyz_way, 1),ang_way(:, 1),interp_points);
angy = interp1(1:size(xyz_way, 1),ang_way(:, 2),interp_points);
angz = interp1(1:size(xyz_way, 1),ang_way(:, 3),interp_points);

clf
% show(robot)
hold on;
xlim([-1 1]);
ylim([-1 1]);
zlim([0 1.5]);
view(135, 20)


goal_xyz = [xx' yy' zz'];
goal_ang = [angx' angy' angz'];


env = {collisionBox(0.3, 0.3, 0.01)};   % default collision object (platform under robot)
env{1}.Pose(3, end) = -0.01;

taskname_add = 'task - sharp path';

end



%% Task 3: Small xyzpry movt at 5 points

function [goal_xyz, goal_ang, env, taskname_add] = taskSmallMovt()

xyz_way = [0.3 0.2 0.6;
           0.4 -0.3 0.8;
           -0.2 -0.3 1;
           -0.2 0.2 1.2;
           0.1 0.4 0.9];

ang_way = [-2.5  0.5  -2.7;
           1.8    1   -0.9;
           0.8  -0.3  -0.4;
          -1.8  -0.7   3;
          -2.5   0.3  -2.8];

n_interp = 10;
interp_points= linspace(1,size(xyz_way, 1), (size(xyz_way, 1)-1)*n_interp+1);
xx = interp1(1:size(xyz_way, 1),xyz_way(:, 1),interp_points, "makima");
yy = interp1(1:size(xyz_way, 1),xyz_way(:, 2),interp_points, "makima");
zz = interp1(1:size(xyz_way, 1),xyz_way(:, 3),interp_points, "makima");
angx = interp1(1:size(xyz_way, 1),ang_way(:, 1),interp_points, "makima");
angy = interp1(1:size(xyz_way, 1),ang_way(:, 2),interp_points, "makima");
angz = interp1(1:size(xyz_way, 1),ang_way(:, 3),interp_points, "makima");

xx2 = []; yy2 = []; zz2 = []; 
angx2 = []; angy2 = []; angz2 = [];

add_at_ind = 1:n_interp:length(interp_points);

ind_ctr = 1;
for jj = 1:length(xx)
    if jj == add_at_ind(ind_ctr)
        for uu = 1:6
            arr_flag = zeros(6,1);
            arr_flag(uu) = 1;
            rot_jj = eul2rotm([angx(jj) angy(jj) angz(jj)], 'XYZ');
            xyz_delta = rot_jj*[.03*arr_flag(1); .03*arr_flag(2); .03*arr_flag(3)];
            xx2 = [xx2, xx(jj), xx(jj)+xyz_delta(1), xx(jj)];
            yy2 = [yy2, yy(jj), yy(jj)+xyz_delta(2), yy(jj)];
            zz2 = [zz2, zz(jj), zz(jj)+xyz_delta(3), zz(jj)];
            angx2 = [angx2, angx(jj), angx(jj)+.3*arr_flag(4), angx(jj)];
            angy2 = [angy2, angy(jj), angy(jj)+.3*arr_flag(5), angy(jj)];
            angz2 = [angz2, angz(jj), angz(jj)+.3*arr_flag(6), angz(jj)];
        end
        ind_ctr = ind_ctr + 1;
    else
        xx2 = [xx2, xx(jj)];
        yy2 = [yy2, yy(jj)];
        zz2 = [zz2, zz(jj)];
        angx2 = [angx2, angx(jj)];
        angy2 = [angy2, angy(jj)];
        angz2 = [angz2, angz(jj)];
    end
end


goal_xyz = [xx2' yy2' zz2'];
goal_ang = [angx2' angy2' angz2'];


env = {collisionBox(0.3, 0.3, 0.01)};   % default collision object (platform under robot)
env{1}.Pose(3, end) = -0.01;

taskname_add = 'task - small movt';

end

%% Task 4: Smooth helical path with gradual changing orientation

function [goal_xyz, goal_ang, env, taskname_add] = taskSmoothHelical()

t = linspace(0, 14*pi, 100);
r = 0.15;
p = 0.02;
X = 0.25+r.*sin(t);
Y = 0.25+r.*cos(t);
Z = 0.3+p.*t;

theta = linspace(0, pi, length(X));
for yy = 1:length(X)
    R = [cos(theta(yy)) -sin(theta(yy)); sin(theta(yy)) cos(theta(yy))];
    newXY = R*[X(yy); Y(yy)];
    X(yy) = newXY(1);
    Y(yy) = newXY(2);
end

xyz_way = [X' Y' Z'];


angx = linspace(pi, pi/2, length(X));
angy = linspace(0, -pi/6, length(X));
angz = linspace(pi, 0, length(X));


goal_xyz = [X', Y', Z'];
goal_ang = [angx' angy' angz'];


env = {collisionBox(0.3, 0.3, 0.01)};   % default collision object (platform under robot)
env{1}.Pose(3, end) = -0.01;

taskname_add = 'task - smooth helical';

end





%% Task 5: Write ICRA facing up and avoiding obstacles

function [goal_xyz, goal_ang, env, taskname_add] = taskICRACollide()

xyz_way = [-0.3 -0.25  1;        % I
           -0.3  0    1;
           -0.3  0.25  1;

           -0.1  0.25  1;       % C
           -0.2  0.1  1;
           -0.2 -0.15  1;
           -0.1 -0.25  1;

            0  -0.25  1;        % R
            0   0  1;
            0  0.25  1;
            0.1  0.25  1;
            0.1  0  1;
            0  0  1;
            0.1  -0.25  1;

            0.15  0  1;         % A
            0.2 0.25 1;
            0.25 0 1;
            0.3 -0.25 1;];

ang_way = zeros(size(xyz_way, 1), 3);

interp_points= linspace(1,size(xyz_way, 1), 100);
xx = interp1(1:size(xyz_way, 1),xyz_way(:, 1),interp_points, "makima");
yy = interp1(1:size(xyz_way, 1),xyz_way(:, 2),interp_points, "makima");
zz = interp1(1:size(xyz_way, 1),xyz_way(:, 3),interp_points, "makima");
angx = interp1(1:size(xyz_way, 1),ang_way(:, 1),interp_points, "makima");
angy = interp1(1:size(xyz_way, 1),ang_way(:, 2),interp_points, "makima");
angz = interp1(1:size(xyz_way, 1),ang_way(:, 3),interp_points, "makima");


wall1 = collisionBox(0.05, 0.65, 0.1);
wall1.Pose = trvec2tform([0.3 0 0.5]);

wall2 = collisionBox(0.65, 0.05, 0.1);
wall2.Pose = trvec2tform([0 0.3 0.5]);

wall3 = collisionBox(0.05, 0.65, 0.1);
wall3.Pose = trvec2tform([-0.3 0 0.5]);

wall4 = collisionBox(0.65, 0.05, 0.1);
wall4.Pose = trvec2tform([0 -0.3 0.5]);


goal_xyz = [xx' yy' zz'];
goal_ang = [angx' angy' angz'];


env = {wall1, wall2, wall3, wall4};

taskname_add = 'task - icra collide';

end
