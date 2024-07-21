close all
clear all 
clc

%% input data
qi = [125 30]; % starting point
qf = [400 135]; % ending point

load('image_map.mat'); % load map

% Plot the image
figure
imshow(image_map,'InitialMagnification','fit');
hold on
scatter(qi(1), qi(2),'g','filled');
scatter(qf(1), qf(2),'b','filled');
hold on

% number of iterations
k = 150;
% incremental distance
dq = 10;

%% RRT algorithm

% init the tree list
tree = qi;
% init the list of point each one of them preceded the tree point
near = qi;

for i = 1 : k 
    % Generate a random configuration on the map
    qr = randomConfiguration(image_map);
    
    % Choosing the nearest tree configuration to the random configuration 
    qNear = nearestConfiguration(qr, tree);
    
    % Generate the new point starting from the nearest configuration
    qNew = newConfiguration(qNear, qr, dq);
    
    % Updating the tree
    [tree, goal, near] = updateTree(qNear, qNew, dq, qf, tree, image_map, near);

    % Plot a red dot for the coordinates at the end of the queue 
    scatter(tree(end,1), tree(end,2), 30, 'r', 'filled'); 
    
    % Plot lines between dots
    plot([near(end,1), tree(end,1)], [near(end,2), tree(end,2)], 'b', 'LineWidth', 1.5); 

    % Add labels
    xlabel('X');
    ylabel('Y');
    title('RRT Path from Start to End');

    if goal == true
        disp('Goal reached!');
        plotEnviroment(tree, qi, qf, dq); 
        return;
    end
    
    hold on
end

if goal == false
    disp('Goal not reached, try to increase the max iteration number');
end

plotEnviroment(tree, qi, qf, dq);


%% Functions definitions

% Function that generates a random point in the map space
function qr = randomConfiguration(map)
    % Picking 2 random coordinates between 1 to row, 1 to column
    qr = [randi(size(map, 2)), randi(size(map, 1))];
end

% Function that pick the nearest point to the random generatet point,
% choosing from the tree elements
function [qNear, normVector] = nearestConfiguration(qr, G)
    % creating the norm vector between each G element and qr
    normVector = zeros(size(G(:,1)));
    for i = 1 : size(G(:,1))
        normVector(i) = norm(G(i,:) - qr);
    end
    % finding the minimum norm
    [~, minIndex] = min(normVector);
    % the closest point is the one with the same index of the minimum norm
    % in the norm vector
    qNear = G(minIndex, :);
end

% Function that generates the new point at distance dq from the nearest
% point in the tree
function [qNew, beta] = newConfiguration(qNear, qr, dq)
    %   qr
    %    |\      tan(beta) = b/c
    %  b | \ a    cos(beta) = c/a 
    %    |  \        sin(beta) = b/a
    %    |___\beta  
    %       c   qNear
    % Computing the angle between qr and qNear
    beta = atan2(qr(2) - qNear(2), qr(1) - qNear(1));
    % Computing the new point coordinates
    qNew = round(qNear + dq * [cos(beta), sin(beta)]);
end

% Function that update the tree with the new point
function [tree, goal, near] = updateTree(qNear, qNew, dq, qf, tree, map, near)
   
    % Initialize goal flag
    goal = false;
    % Checking if the point is inside the map boundaries and not colliding with obstacles
    if qNew(1) >= 1 && qNew(1) <= size(map, 2) && ...
       qNew(2) >= 1 && qNew(2) <= size(map, 1) && ...
       map(qNew(2), qNew(1)) == 1 && ...
       collisionDetector(map, qNear, qNew)
       % Add the new point to the tree
       tree = [tree; qNew];
       % Store the nearest point to the actual tree point 
       near = [near; qNear];
       % Check if the new point is close to the goal
       if norm(qNew - qf) < dq
           goal = true;
           return;
       end
    end
end

% Function that detect if there arey collision in the rectangular area
% between the previous and actual point
function collisionFree = collisionDetector(map, qNear, qNew)
    % Checking if all the blocks inside the rectangular area between qNear
    % and qNew are 1, so without obstacles
    collisionFree = all(map(min(qNew(2), qNear(2)):max(qNew(2), qNear(2)),...
        min(qNew(1), qNear(1)):max(qNew(1), qNear(1))) == 1, 'all');

end

% Plot function
function plotEnviroment(tree, qi, qf, dq)
        % Plot starting and final points with the goal circumference
        rectangle('Position', [qf(1) - dq, qf(2) - dq, 2*dq, 2*dq], ...
                  'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 1, ...
                  'LineStyle', '--');  % shaping the rectangular as a circle
        scatter(qi(1), qi(2),'g','filled');
        scatter(qf(1), qf(2),'y','filled');
        scatter(tree(end,1), tree(end,2), 'b','filled') % final tree point
        % Plot the line distance between the last tree point and the goal
        plot([tree(end,1) qf(1)], [tree(end,2) qf(2)], 'b', 'LineWidth', 1.5, ...
            'LineStyle', '--');
end
