function RRT_Star_Path_Planning
    %% Main GUI Setup
    fig = uifigure('Name', 'RRT* Path Planning GUI', 'Position', [50 50 1200 750]);

    pnl = uipanel(fig, 'Title', 'Parameters and Obstacles', ...
        'Position', [20 20 460 710], ...
        'FontWeight', 'bold', 'FontSize', 12);

    sectionStyle = {'FontWeight', 'bold', 'FontSize', 11, 'FontColor', [0.1 0.3 0.6]};

    %% Start & Goal Inputs
    uilabel(pnl, 'Text', 'Start and Goal Positions', 'Position', [10 700 330 22], sectionStyle{:});
    uilabel(pnl, 'Text', 'Start X:', 'Position', [20 660 50 22]);
    startX = uieditfield(pnl, 'numeric', 'Position', [80 660 80 22], 'Value', 0);
    uilabel(pnl, 'Text', 'Start Y:', 'Position', [180 660 50 22]);
    startY = uieditfield(pnl, 'numeric', 'Position', [240 660 80 22], 'Value', 0);

    uilabel(pnl, 'Text', 'Goal X:', 'Position', [20 620 50 22]);
    goalX = uieditfield(pnl, 'numeric', 'Position', [80 620 80 22], 'Value', 1000);
    uilabel(pnl, 'Text', 'Goal Y:', 'Position', [180 620 50 22]);
    goalY = uieditfield(pnl, 'numeric', 'Position', [240 620 80 22], 'Value', 1000);

    %% Algorithm Parameters
    uilabel(pnl, 'Text', 'Algorithm Parameters', 'Position', [10 570 330 22], sectionStyle{:});
    uilabel(pnl, 'Text', 'Step Size (EPS):', 'Position', [20 530 100 22]);
    epsField = uieditfield(pnl, 'numeric', 'Position', [130 530 50 22], 'Value', 200);
    uilabel(pnl, 'Text', 'Max Nodes:', 'Position', [20 490 100 22]);
    nodesField = uieditfield(pnl, 'numeric', 'Position', [130 490 60 22], 'Value', 3000);

    ax1 = uiaxes(fig, 'Position',[500 50 670 300]); %[500 400 350 300]
    ax1.Title.String = 'RRT* Path Planning';
    ax1.XLabel.String = 'X';
    ax1.YLabel.String = 'Y';
    ax1.XLim = [0 1000];
    ax1.YLim = [0 1000];
    ax1.Box = 'on';
    ax1.GridLineStyle = ':';

    ax2 = uiaxes(fig, 'Position', [870 400 300 300]);
    ax2.Title.String = 'Cumulative Distance';
    ax2.XLabel.String = 'Index';
    ax2.YLabel.String = 'Distance';
    ax2.Box = 'on';
    ax2.GridLineStyle = ':';

    ax3 = uiaxes(fig, 'Position', [500 400 350 300]);%[500 50 670 300]
    ax3.Title.String = 'Direct vs RRT* Path';
    ax3.XLabel.String = 'X';
    ax3.YLabel.String = 'Y';
    ax3.XLim = [0 1000];
    ax3.YLim = [0 1000];
    ax3.Box = 'on';
    ax3.GridLineStyle = ':';

    %% Obstacle Section
    uilabel(pnl, 'Text', 'Obstacle Configuration', 'Position', [10 460 330 22], sectionStyle{:});
    uilabel(pnl, 'Text', '[X    Y    Width    Height]', 'Position', [20 440 260 22], 'FontAngle', 'italic');

    obsDefault = [
        0 700 200 200;
        100 400 200 200;
        300 100 200 200;
        400 700 200 200;
        600 300 200 200;
        800 700 200 100;
        500 100 200 100;
        100 600 200 100;
        200 200 200 100;
        0 300 200 100;
        500 400 100 200;
        600 600 100 200;
        300 800 100 200;
        700 100 100 200;
        900 400 100 200];

    obsFields = gobjects(15,4);
    ypos = 420;

    for i = 1:15
        for j = 1:4
            obsFields(i,j) = uieditfield(pnl, 'numeric', ...
                'Position', [20+(j-1)*100, ypos, 80, 22], ...
                'Value', obsDefault(i,j));
        end
        ypos = ypos - 28;
    end

    %% Buttons Section
    uibutton(pnl, 'Text', 'Run ', 'Position',[10 5 100 22] , ...
        'FontSize', 11, 'FontWeight', 'bold', ...
        'ButtonPushedFcn', @(btn,event) runRRT_GUI( ...
            startX.Value, startY.Value, goalX.Value, goalY.Value, ...
            obsFields, epsField.Value, nodesField.Value, ...
            ax1, ax2, ax3));
    uibutton(pnl, 'Text', 'Reset', 'Position', [130 5 100 22], ...
        'ButtonPushedFcn', @(btn,event) resetAxes([ax1,ax2,ax3]));
end

function resetAxes(axesArray)
    for ax = axesArray
        cla(ax); title(ax,''); xlabel(ax,''); ylabel(ax,'');
    end
end

function runRRT_GUI(sx,sy,gx,gy,obsFields,EPS,numNodes,ax1,ax2,ax3)
    x_max = 1000; y_max = 1000;
    qstart.coord=[sx sy]; qstart.cost=0; qstart.parent=0;
    qgoal.coord=[gx gy]; qgoal.cost=0;

    % Read obstacles
    obstacle = zeros(15,4);
    for i = 1:15
        for j = 1:4
            obstacle(i,j) = obsFields(i,j).Value;
        end
    end

    axes(ax1); cla(ax1); hold(ax1,'on'); axis(ax1,[0 x_max 0 y_max]);
    for i = 1:size(obstacle,1)
        rectangle(ax1, 'Position', obstacle(i,:), 'FaceColor', "k");
    end
    plot(ax1, sx, sy, 'go', 'MarkerFaceColor','g');
    plot(ax1, gx, gy, 'ro', 'MarkerFaceColor','r');
    title(ax1, 'RRT* Path Planning');

    % Check goal inside obstacle
    for i=1:size(obstacle,1)
        o = obstacle(i,:);
        if gx>=o(1)&&gx<=o(1)+o(3)&&gy>=o(2)&&gy<=o(2)+o(4)
            title(ax1,'Goal is inside obstacle!');
            return;
        end
    end

    nodes = qstart;
    goal_reached = false;

    for i = 1:numNodes

        if rand < 0.1
            q_rand = [gx gy];
        else
            theta = rand*2*pi;
            r = EPS*sqrt(rand);
            q_rand = nodes(end).coord + r*[cos(theta), sin(theta)];
        end
            q_rand = max([0,0], min([x_max,y_max], q_rand));
        
        % Find nearest node
        dists = arrayfun(@(node) distance(node.coord, q_rand), nodes);
        [val , idx] = min(dists);
        q_near = nodes(idx);


        q_new.coord = steer(q_rand, q_near.coord, val, EPS);

        if noCollision(q_near.coord, q_new.coord, obstacle)
            line(ax1,[q_near.coord(1) q_new.coord(1)], [q_near.coord(2) q_new.coord(2)], 'Color','k');
            q_new.cost = q_near.cost + distance(q_near.coord, q_new.coord);

            neighbor_radius =300;
            min_cost = q_new.cost;
            best_parent = idx;

            for j = 1:numel(nodes)
                d = distance(nodes(j).coord, q_new.coord);
                if d <= neighbor_radius && noCollision(nodes(j).coord, q_new.coord, obstacle)
                    temp_cost = nodes(j).cost + d;
                    if temp_cost < min_cost
                        min_cost = temp_cost;
                        best_parent = j;
                        line(ax1,[nodes(j).coord(1) q_new.coord(1)], [nodes(j).coord(2) q_new.coord(2)], 'Color','g');
                    end
                end
            end

            q_new.parent = best_parent;
            nodes(end+1) = q_new;

            if distance(q_new.coord, [gx gy]) < EPS && noCollision(q_new.coord, [gx gy], obstacle)
                qgoal.parent = numel(nodes);
                nodes(end+1) = qgoal;
                goal_reached = true;
                break;
            end
        end
    end

    if ~goal_reached
        title(ax1,'Failed to Reach Goal');
        return;
    end

    % Reconstruct path
    path = qgoal.coord;
    q = qgoal;
    while q.parent ~= 0
        pidx = q.parent;
        line(ax1, [q.coord(1), nodes(pidx).coord(1)], [q.coord(2), nodes(pidx).coord(2)], 'Color','r','LineWidth',2);
        q = nodes(pidx);
        path = [q.coord; path];
    end
    disp('Path found successfully!');

    % Cumulative distance
    axes(ax2); cla(ax2);
    cumdist = zeros(size(path,1),1);
    for k = 2:size(path,1)
        cumdist(k) = cumdist(k-1) + distance(path(k-1,:), path(k,:));
    end
    plot(ax2, 1:length(cumdist), cumdist,'b-o','LineWidth',2);
    title(ax2,'Cumulative Distance from Start');

    % Compare direct vs RRT* path
    axes(ax3); cla(ax3); hold(ax3,'on');
    plot(ax3, sx,sy,'go','MarkerFaceColor','g');
    plot(ax3, gx,gy,'ro','MarkerFaceColor','r');
    line(ax3, [sx gx], [sy gy], 'Color','b','LineStyle','--','LineWidth',2);
    rrtD = 0; q = qgoal; directD = distance(qstart.coord , qgoal.coord);
    while q.parent ~= 0
        pidx = q.parent;
        line(ax3, [q.coord(1), nodes(pidx).coord(1)], [q.coord(2), nodes(pidx).coord(2)], 'Color','r','LineWidth',2);
        rrtD = rrtD + distance(q.coord, nodes(pidx).coord);
        q = nodes(pidx);
    end
     legend(ax3, {'Start','Goal','Direct Path','RRT* Path'}, 'Location','bestoutside');
    title(ax3,'Direct vs RRT* Path');
end

function p_new = steer(p_rand, p_near, r, Expansion)
    dx = p_rand(1)-p_near(1);
    dy = p_rand(2)-p_near(2);
    if r > Expansion
        theta = atan2(dy,dx);
        p_new = p_near + Expansion*[cos(theta), sin(theta)];
    else
        p_new = p_rand;
    end
    p_new = round(p_new);
end

function d = distance(p1, p2)
    d = sqrt(sum((p1-p2).^2));
end

function collisionFree = noCollision(p1, p2, obstacles)
    collisionFree = true;

    x1 = p1(1); y1 = p1(2);
    x2 = p2(1); y2 = p2(2);

    nSteps = 200;

    if x1 == x2
        % Vertical line — interpolate over y instead
        y_values = linspace(min(y1, y2), max(y1, y2), nSteps);
        x = x1 * ones(size(y_values));
        points = [x(:), y_values(:)];
    else
        % Linear interpolation over x
        x_values = linspace(min(x1, x2), max(x1, x2), nSteps);
        y_values = ((x_values - x2) ./ (x1 - x2)) * y1 + ((x_values - x1) ./ (x2 - x1)) * y2;
        points = [x_values(:), y_values(:)];
    end

    % Check each point for collision
    for k = 1:size(points, 1)
        pt = points(k, :);
        for i = 1:size(obstacles,1)
            o = obstacles(i,:); % [x y w h]
            if pt(1) >= o(1) && pt(1) <= o(1)+o(3) && ...
               pt(2) >= o(2) && pt(2) <= o(2)+o(4)
                collisionFree = false;
                return;
            end
        end
    end
end
