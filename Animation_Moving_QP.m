clear all
clc

H = 0;
%% Import selected varaibles from .mat file to workspace
load('Scenarios/scenario10.mat', 'Pobs1x', 'Pobs1y','Pobs2x','Pobs2y','X', 'X_dot', 'Y', 'Y_dot', 'xd', 'yd', 'd', 'r','H');

%% Graphical settings

% Resize to full screen
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 1]);

% Define graph limits
xmin = -10;
xmax = 50;
ymin = 0;
ymax = 50;

xlim([xmin, xmax]);
ylim([ymin, ymax]);

% MarkerSize
markerSizeInAxesUnits = 3;


hold on
axis equal

ax = gca;

h = hgtransform('Parent',ax);
j = hgtransform('Parent',ax);

%% Create Animation objects

% Real robot current position
pos_real = rectangle('Position', [xd(1)-r, xd(2), 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0.9290 0.6940 0.1250 0.5],...
                      'Parent', h);


% Reference robot current position
pos_ref = rectangle('Position', [X(1)-r, Y(1)-r, 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0 0.5 1 0.5],...
                      'Parent', j);

% Obstacles visualization
Obs1 = rectangle('Position', [Pobs1x(1)-r, Pobs1y(1)-r, 2*d, 2*d],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [1 0 0 0.9],...
                      'Parent', j);

Obs2 = rectangle('Position', [Pobs2x(1)-d, Pobs2y(1)-d, 2*d, 2*d],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [1 0 0 0.9],...
                      'Parent', j);


% Create Trail effect for the real and reference position
trail_real = line('XData', [], 'YData', [], 'Color', [0.9290 0.6940 0.1250, 0.5], 'LineWidth', 2, 'Parent', h);
trail_ref = line('XData', [], 'YData', [], 'Color', [0 0.5 1, 0.5], 'LineWidth', 2, 'Parent', j);

% Create istantaneous velocity v_vectortor for real current position
v_vector = quiver(X(1), Y(1), X_dot(1), Y_dot(1), 'Color', 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
hold off

% Compute current velocity magnitude
vel =  sqrt(X_dot.^2 + Y_dot.^2);

% Display Current Velocity
v_text = text(X(1),Y(1),num2str(vel(1)),'Parent',j,...
'VerticalAlignment','top','FontSize',14);


%% Start animation
for k = 2:length(xd)
    % Update robot and obstacles positions
    set(pos_real, 'Position', [xd(k)-r, yd(k)-r, 2*r, 2*r]);
    set(pos_ref, 'Position', [X(k)-r, Y(k)-r, 2*r, 2*r]);
    set(Obs1, 'Position', [Pobs1x(k)-d, Pobs1y(k)-d, 2*d, 2*d]);
    set(Obs2, 'Position', [Pobs2x(k)-d, Pobs2y(k)-d, 2*d, 2*d]);
    
    % Update trail effects
    x_trail_real = [get(trail_real, 'XData'), xd(k)];
    y_trail_real = [get(trail_real, 'YData'), yd(k)];
    x_trail_ref = [get(trail_ref, 'XData'), X(k)];
    y_trail_ref = [get(trail_ref, 'YData'), Y(k)];
    
    set(trail_real, 'XData', x_trail_real, 'YData', y_trail_real);
    set(trail_ref, 'XData', x_trail_ref, 'YData', y_trail_ref);
    
    % Update current velocity magnitude displayed
    if H
        set(v_text, 'Position', [X(k), Y(k)], 'String', num2str(H(k)-(r+d)));
    end
    % Update current velocity direction
    set(v_vector, 'XData', X(k), 'YData', Y(k), 'UData', X_dot(k), 'VData', Y_dot(k));
    
    xlim([-10, 50]);
    ylim([0, 50]);

    % Generate frame
    drawnow
end