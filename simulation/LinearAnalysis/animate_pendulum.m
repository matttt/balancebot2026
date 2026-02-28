function animate_pendulum(tilt, yaw, distance, dt)
% ANIMATE_PENDULUM Animates a two-wheeled inverted pendulum (Segway-like).
%
%   Usage:
%       animate_pendulum(tilt, yaw, distance, dt)
%       animate_pendulum() % Runs a demo
%
%   Inputs:
%       tilt     - Vector of body tilt angles (radians)
%       yaw      - Vector of heading angles (radians)
%       distance - Vector of distance traveled (meters)
%       dt       - Time step between frames (seconds) [Optional, default 0.05]
%
%   Coordinate Frame:
%       X: Forward, Y: Left, Z: Up.
%       Yaw is rotation around Z. Tilt is rotation around local Y (axle).

    % --- 1. Handle Demo Mode and Defaults ---
    if nargin == 0
        disp('No inputs provided. Running Demo Mode...');
        [tilt, yaw, distance, dt] = generate_demo_data();
    end
    
    if nargin < 4
        dt = 0.05; % Default 20 FPS
    end

    % --- 2. Physical Parameters ---
    r_wheel = 0.15;   % Radius of wheels
    w_wheel = 0.05;   % Width of wheels
    w_axle  = 0.5;    % Distance between wheels
    h_body  = 1.0;    % Height of the pendulum stick/body
    
    % --- 3. Pre-calculate Global Position (Path Integration) ---
    % We convert distance/yaw into X/Y coordinates
    N = length(distance);
    pos_x = zeros(N, 1);
    pos_y = zeros(N, 1);
    
    for i = 2:N
        dd = distance(i) - distance(i-1);
        % Use average yaw for better integration accuracy
        avg_yaw = yaw(i); 
        pos_x(i) = pos_x(i-1) + dd * cos(avg_yaw);
        pos_y(i) = pos_y(i-1) + dd * sin(avg_yaw);
    end

    % --- 4. Setup Figure and Environment ---
    f = figure('Color', 'w', 'Name', 'Inverted Pendulum Animation');
    ax = axes('Parent', f);
    axis equal; grid on; hold on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    view(30, 30); % Initial camera angle
    
    % Set axis limits dynamically based on path
    buffer = 1;
    xlim([min(pos_x)-buffer, max(pos_x)+buffer]);
    ylim([min(pos_y)-buffer, max(pos_y)+buffer]);
    zlim([0, h_body + 0.5]);

    % Draw the ground path trace
    plot3(pos_x, pos_y, zeros(size(pos_x)), 'k--', 'LineWidth', 1);

    % --- 5. Create Geometry Objects ---
    
    % Parent transform: Moves the whole robot (Translation + Yaw)
    t_robot = hgtransform('Parent', ax);
    
    % Child transform: Body Tilt (Rotates around axle)
    t_body_tilt = hgtransform('Parent', t_robot);
    
    % Child transforms: Wheels (Spin around axle)
    t_wheel_L = hgtransform('Parent', t_robot);
    t_wheel_R = hgtransform('Parent', t_robot);

    % Geometry: Wheel (Cylinder)
    [Xc, Yc, Zc] = cylinder(r_wheel, 20);
    Zc = Zc * w_wheel; % Scale width
    % Center wheel on its local frame
    Zc = Zc - w_wheel/2; 
    
    % Draw Left Wheel
    wheel_color = [0.2 0.2 0.2];
    p_wheel_L = patch(surf2patch(Xc, Zc, Yc), 'Parent', t_wheel_L, ...
        'FaceColor', wheel_color, 'EdgeColor', 'none'); 
    % Note: Swapped Y/Z in surf2patch to align cylinder along Y-axis
    
    % Draw Right Wheel
    p_wheel_R = patch(surf2patch(Xc, Zc, Yc), 'Parent', t_wheel_R, ...
        'FaceColor', wheel_color, 'EdgeColor', 'none');

    % Geometry: Body (Box/Rod)
    bw = 0.1; % Body width
    % Define vertices for a simple box
    % Creating a rod centered at bottom
    [Xb, Yb, Zb] = cylinder([0.02 0.02], 8); 
    Zb = Zb * h_body;
    % Draw Body
    patch(surf2patch(Xb, Yb, Zb), 'Parent', t_body_tilt, ...
        'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none');
    
    % Geometry: Axle (connects wheels)
    [Xa, Ya, Za] = cylinder(0.01, 8);
    Za = Za * w_axle - w_axle/2;
    patch(surf2patch(Xa, Za, Ya), 'Parent', t_robot, ...
        'FaceColor', 'k', 'EdgeColor', 'none');

    % Geometry: A "Head" to see orientation
    [Xs, Ys, Zs] = sphere(10);
    r_head = 0.08;
    patch(surf2patch(Xs*r_head, Ys*r_head, Zs*r_head + h_body), ...
        'Parent', t_body_tilt, 'FaceColor', 'b', 'EdgeColor', 'none');

    % Light to make it look 3D
    camlight('headlight'); lighting gouraud;

    % --- 6. Animation Loop ---
    
    % Offset wheels to their positions relative to robot center
    % Left is +Y, Right is -Y (or vice versa depending on frame)
    offset_L = makehgtform('translate', [0, w_axle/2, 0]);
    offset_R = makehgtform('translate', [0, -w_axle/2, 0]);
    
    disp('Starting animation...');
    
    for k = 1:N
        if ~isvalid(f), break; end % Stop if window closed
        
        % 1. Global Position & Yaw
        % Move robot to (x, y) and up by wheel radius (so wheels touch ground)
        trans = makehgtform('translate', [pos_x(k), pos_y(k), r_wheel]);
        rot_yaw = makehgtform('zrotate', yaw(k));
        set(t_robot, 'Matrix', trans * rot_yaw);
        
        % 2. Body Tilt
        % Rotate around local Y axis
        % Note: We add pi/2 or similar if the cylinder definition needs offset
        % Here we assume tilt=0 is vertical.
        rot_tilt = makehgtform('yrotate', tilt(k)); 
        set(t_body_tilt, 'Matrix', rot_tilt);
        
        % 3. Wheel Spin
        % Angle = distance / radius
        wheel_angle = distance(k) / r_wheel;
        % Wheels rotate around Y axis. 
        % Note: +x movement requires -y rotation (right hand rule) or similar
        spin = makehgtform('yrotate', -wheel_angle); 
        
        set(t_wheel_L, 'Matrix', offset_L * spin);
        set(t_wheel_R, 'Matrix', offset_R * spin);
        
        % 4. Follow Camera (Optional - uncomment to track robot)
        % cx = pos_x(k) - 2*cos(yaw(k));
        % cy = pos_y(k) - 2*sin(yaw(k));
        % camtarget([pos_x(k), pos_y(k), h_body/2]);
        % campos([cx, cy, 2]);

        drawnow;
        
        % Simple timing control
        pause(dt); 
    end
    disp('Animation complete.');
end

function [tilt, yaw, dist, dt] = generate_demo_data()
    % Generates a Figure-8 path with realistic tilting
    t = 0:0.05:20;
    dt = 0.05;
    
    % Path: Figure 8
    % x = sin(t), y = sin(t)*cos(t)
    % But we need Distance and Yaw inputs, not X/Y directly.
    
    % Let's synthesize Tilt/Yaw directly
    % Swing back and forth
    tilt = 0.2 * sin(2*t); 
    
    % Rotate in a circle
    yaw = 0.5 * t; 
    
    % Move forward with varying speed
    velocity = 1.0 + 0.5 * sin(t);
    dist = cumsum(velocity * dt);
end