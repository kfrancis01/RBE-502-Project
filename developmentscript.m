clear all
close all

theta = pi/4;
cup_x = 0.3;
cup_y = 0.3;
cup_radius = 0.01;
cup_front = cup_x - cup_radius;
cup_back = cup_x + cup_radius;

% ball_at_release = [x, y , xv, yv]
ball_at_release = ball_velocity(theta, cup_x, cup_y);
x = ball_at_release(1);
y = ball_at_release(2);
xv = ball_at_release(3);
yv = ball_at_release(4);
total_time = 1;

pose = ball_traj(x, y, xv, yv, total_time);

plot(pose(:,1), pose(:,2));  % x vs y
axis equal;
grid on;
hold on;
xlabel('X Position');
ylabel('Y Position');
title('Ball Trajectory');
plot(cup_x, cup_y, 'x', 'Color', 'r', 'MarkerSize', 10);
plot(cup_front, cup_y, '|', 'Color', 'k', 'MarkerSize', 10);
plot(cup_back, cup_y, '|', 'Color', 'k', 'MarkerSize', 10);

num_points = 0;
for i = 1:length(pose(:, 2))
    if num_points < 1
        % if any of our y vals are zero then plot a point max of 2 points
        
        if (pose(i,2) <= cup_y)
            x_on_target = tolerance(pose(i,1), cup_x, cup_radius);
            if x_on_target
                plot(pose(i,1), pose(i,2), 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
                num_points = num_points + 1;
            end
        end
    end
end
save('ballPose.mat','ball_at_release','pose','ee_traj');


function pose = ball_velocity(theta, cup_x, cup_y)
% returns the pose of the ball at release point
% pose = [x, yv , xv, yv]
%    theta = pi/4;
    robot_release_x = 0;
    robot_release_y = 0.5;
%     cup_x = 10;
%     cup_y = 1;
    g = 9.81;
    a = (sin(theta)*cup_x/cos(theta)) - (cup_y - robot_release_y);
    vel = sqrt((cup_x^2*g)/(2*cos(theta)^2*a));
    x_vel = vel*cos(theta);
    y_vel = vel*sin(theta);
    pose = [robot_release_x, robot_release_y, x_vel, y_vel];
end


function pose = ball_traj(x_i, y_i, xv_i, yv_i,total_time)
    g = -9.81;
    dt = 0.001;
    time = 0:dt:total_time;
    x_pose = [];
    y_pose = [];

    release = false;
    release_time = 0;
    y_release = 0;
    yv_release = 0;

    for i = 1:length(time)
        ti = time(i);
        % robot holding ball 
        % x is EE x pose
        x = xv_i * ti + x_i;

        if ~release && x > 0
            % stores y vel and pose when we release
            release = true;
            release_time = ti;
            y_release = yv_i * ti + y_i;
            yv_release = yv_i;
        end

        % robot holding ball phase
        if ~release
            % if ball is in robot hand
            % y velcoity is ee y velocity
            y = yv_i * ti + y_i;
        else
            % once robot releases at x = 0  ball in free fall
            t_since_release = ti - release_time;
            y = y_release + yv_release * t_since_release + 0.5 * g * t_since_release^2;
        end
        % if ball is on ground
        if y <= 0
            y = 0;
        end
        x_pose = [x_pose; x];
        y_pose = [y_pose; y];
    end
    pose = [x_pose , y_pose];

end

function on_target = tolerance(point, goal, some_tolerance)
    upper = goal + some_tolerance;
    lower = goal - some_tolerance;
    if (lower <= point) && (point <= upper)
        on_target = true;
    else
        on_target = false;
    end
end





