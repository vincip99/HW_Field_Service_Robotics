clear all
close all
clc

%% Defining Data
% Maps definition
map_0 = [0 -1 -1  0  0  0  0  0  0  0;
         0  0 -1  0  0  0  0 -1 -1  0;
         0  0 -1  0 -1  0  0 -1  0  0;
         0  0 -1  0 -1  0  0 -1  0  0;
         0  0 -1  0 -1 -1  0 -1  0  0;
         0  0  0  0  0  0  0  0  0  0;
         0  0  0  0  0  0 -1  0  0  0;];

% Used only to compare
% map_8 = [0 -1 -1 10 10 10 11 12 13 14;
%         1 1  -1 9  9  10 11 -1 -1 14;
%         2 2  -1 8  -1 10 11 -1 13 13;
%         3 3  -1 7  -1 10 10 -1 12 12;
%         4 4  -1 6  -1 -1 9  -1 11 12;
%         5 5  5  6  7  8  9  10 11 12;
%         6 6  6  6  7  8  -1 10 11 12;];
% 
% map_4 = [0 -1 -1  13 14  15  16  17  18  19;
%         1  2 -1  12 13  14  15  -1  -1  18;
%         2  3 -1  11 -1  15  14  -1  16  17;
%         3  4 -1  10 -1  14  13  -1  15  16;
%         4  5 -1  9  -1  -1  12  -1  14  15;
%         5  6  7  8   9  10  11  12  13  14;
%         6  7  8  9  10  11  -1  13  14  15;];

qg = [1 1]; % Goal point coordinates
qs = [1 8]; % Starting point coordinates

%% Path finding 

% plot empty map
figure
imagesc(map_0);
colormap(gray);
im.AlphaData = 0.5;
grid on;
axis image;
title('Empty map')

% generate the first map
map1 = breadth4explorer(map_0,qg);
% generate the second map
map2 = breadth8explorer(map_0,qg);

figure
% plotting maps
subplot(2,2,1)
imagesc(map1);
colormap(gray);
im.AlphaData = 0.5;
grid on;
axis image;
title('4 adjacent cell map')

subplot(2,2,2)
imagesc(map2);
colormap(gray);
im.AlphaData = 0.5;
grid on;
axis image;
title('8 adjacent cell map')

% Path finding and cost 
path1dfs = dfs(map1, qs, qg)
[hop1dfs, cost1dfs] = hopCounter(map1, path1dfs)

path2dfs = dfs(map2, qs, qg)
[hop2dfs, cost2dfs] = hopCounter(map2, path2dfs)

path1 = depthFirstHeuristic(map1, qs, qg)
[hop1, cost1] = hopCounter(map1, path1)

path2 = depthFirstHeuristic(map2, qs, qg)
[hop2, cost2] = hopCounter(map2, path2)

% Plot the first path
subplot(2,2,3)
imagesc(map1);
colormap(gray);
im.AlphaData = 0.5;
grid on;
axis image;
hold on
% Plot starting and ending point
rectangle('Position', [qs(2)-0.5, qs(1)-0.5, 1, 1], ...
        'FaceColor', 'b', 'EdgeColor', 'none');
rectangle('Position', [qg(2)-0.5, qg(1)-0.5, 1, 1], ...
        'FaceColor', 'r', 'EdgeColor', 'none');
hold on
title('Path Finding Algorithm depthFirstHeuristic')
for i = 2:size(path1, 1)-1
    pause(0.2);
    rectangle('Position', [path1(i, 2)-0.5, path1(i, 1)-0.5, 1, 1], ...
        'FaceColor', 'y', 'EdgeColor', 'none');
end
hold off


% Plot the second path
subplot(2,2,4)
imagesc(map2);
colormap(gray);
im.AlphaData = 0.5;
grid on;
axis image;
hold on
% Plot starting and ending point
rectangle('Position', [qs(2)-0.5, qs(1)-0.5, 1, 1], ...
        'FaceColor', 'b', 'EdgeColor', 'none');
rectangle('Position', [qg(2)-0.5, qg(1)-0.5, 1, 1], ...
        'FaceColor', 'r', 'EdgeColor', 'none');
hold on
title('Path Finding Algorithm depthFirstHeuristic')
for i = 2:size(path2, 1)-1
    pause(0.2);
    rectangle('Position', [path2(i, 2)-0.5, path2(i, 1)-0.5, 1, 1], ...
        'FaceColor', 'y', 'EdgeColor', 'none');
end
hold off

figure
% Plot the first dfs path
subplot(1,2,1)
imagesc(map1);
colormap(gray);
im.AlphaData = 0.5;
grid on;
axis image;
hold on
% Plot starting and ending point
rectangle('Position', [qs(2)-0.5, qs(1)-0.5, 1, 1], ...
        'FaceColor', 'b', 'EdgeColor', 'none');
rectangle('Position', [qg(2)-0.5, qg(1)-0.5, 1, 1], ...
        'FaceColor', 'r', 'EdgeColor', 'none');
hold on
title('Path Finding Algorithm 4 adjacent dfs')
for i = 2:size(path1dfs, 1)-1
    pause(0.2);
    rectangle('Position', [path1dfs(i, 2)-0.5, path1dfs(i, 1)-0.5, 1, 1], ...
        'FaceColor', 'y', 'EdgeColor', 'none');
end
hold off


% Plot the second dfs path
subplot(1,2,2)
imagesc(map2);
colormap(gray);
im.AlphaData = 0.5;
grid on;
axis image;
hold on
% Plot starting and ending point
rectangle('Position', [qs(2)-0.5, qs(1)-0.5, 1, 1], ...
        'FaceColor', 'b', 'EdgeColor', 'none');
rectangle('Position', [qg(2)-0.5, qg(1)-0.5, 1, 1], ...
        'FaceColor', 'r', 'EdgeColor', 'none');
hold on
title('Path Finding Algorithm 8 adjacent dfs')
for i = 2:size(path2dfs, 1)-1
    pause(0.2);
    rectangle('Position', [path2dfs(i, 2)-0.5, path2dfs(i, 1)-0.5, 1, 1], ...
        'FaceColor', 'y', 'EdgeColor', 'none');
end
hold off


%% Functions definitions
% floading algorithm with 4 adjacent cells
function [map, path] = breadth4explorer(emptyMap, qg)
    
    % Init the queue and the visited vector
    row = qg(1); 
    col = qg(2);
    queue = [row, col];
    visited = [];
    
    % iteration until the queue has no more unexplored points
    while size(queue,1) > 0 
        % queue pop
        row = queue(1, 1);
        col = queue(1, 2);
        queue(1,:) = [];
        % marking the point as visited
        visited = [visited; [row, col]];
        % searching the four adjacent cells
        list = fourNeighbors(emptyMap, row, col);
        % for every cell in list modify the respective value in the map
        for i = 1 : size(list,1)
            % if the cell was not yet visited
            if (~ismember(visited(:,1),list(i,1))) | (~ismember(visited(:,2),list(i,2)))
                % floading the 4 adjacent cells 
                emptyMap(list(i,1),list(i,2)) = emptyMap(row, col) + 1; 
                visited = [visited; list(i,1),list(i,2)]; % adding the cells to visited
                queue = [queue; [list(i,1), list(i,2)]]; % update the queue
            end
        end
    end
    map = emptyMap; 
    path = visited;
end

% floading algorithm with 8 adjacent cells
function [map, path] = breadth8explorer(emptyMap, qg)

    row = qg(1); 
    col = qg(2);
    queue = [row, col];
    visited = [];

    while size(queue,1) > 0 
        row = queue(1, 1);
        col = queue(1, 2);
        queue(1,:) = [];
        visited = [visited; [row, col]];
        list = eightNeighbors(emptyMap, row, col);
        for i = 1 : size(list,1)
            if (~ismember(visited(:,1),list(i,1))) | (~ismember(visited(:,2),list(i,2)))
                emptyMap(list(i,1),list(i,2)) = emptyMap(row, col) + 1;
                visited = [visited; list(i,1),list(i,2)];
                queue = [queue; [list(i,1), list(i,2)]];
            end
        end
    end
    map = emptyMap; 
    path = visited;
end

% path finding algorithm with heuristic stack sort
function [path, stack, heuristic] = depthFirstHeuristic(map, qs, qg)
    % Init the stack and the visited vector
    row = qs(1); 
    col = qs(2);
    stack = [row, col];
    visited = [];
    
    %until we reach the goal
    while (row ~= qg(1) || col ~= qg(2))
        % pop
        row = stack(end,1);
        col = stack(end,2);
        stack(end,:) = [];
        % marking the point as visited
        visited = [visited; [row, col]];

        % searching the 8 adjacent cells with
        list = neighbors(map, row, col, -1);
        % using the heuristic to sort the stack in order to pick first the
        % desend order list
        heuristic = heuristicSorting(map, list);
        % for each valid cell update the stack
        for i = 1 : size(list,1)
            if (~ismember(visited(:,1),heuristic(i,1))) | (~ismember(visited(:,2),heuristic(i,2)))
                stack = [stack; [heuristic(i,1), heuristic(i,2)]];
            end
        end
    end
    path = visited;
end

% adjacent cell list function
function list = neighbors(map, row, col, wallValue)
        list = [];
        % adding neighbors row and column value only if they are not wall
        % or out of the map
        if row + 1 <= size(map,1) && map(row + 1, col) ~= wallValue
            list = [list; [row + 1, col]];
        end
        if row - 1 >= 1 && map(row - 1, col) ~= wallValue
            list = [list; [row - 1, col]];
        end
        if col + 1 <= size(map,2) && map(row, col + 1) ~= wallValue
            list = [list; [row, col + 1]];
        end
        if col - 1 >= 1 && map(row, col - 1) ~= wallValue
            list = [list; [row, col - 1]];
        end
        if row - 1 >= 1 && col - 1 >= 1 && map(row - 1, col - 1) ~= wallValue
            list = [list; [row - 1, col - 1]];
        end
        if row - 1 >= 1 && col + 1 <= size(map,2) && map(row - 1, col + 1) ~= wallValue
            list = [list; [row - 1, col + 1]];
        end
        if row + 1 <= size(map,1) && col - 1 >= 1 && map(row + 1, col - 1) ~= wallValue
            list = [list; [row + 1, col - 1]];
        end
        if row + 1 <= size(map,1) && col + 1 <= size(map,2) && map(row + 1, col + 1) ~= wallValue
            list = [list; [row + 1, col + 1]];
        end
end

% heuristic stack sort
function heuristic = heuristicSorting(map, list)
    
    valueList = zeros(size(list,1));
    heuristic = zeros(size(list,2));
    for i = 1 :size(list,1)
        valueList(i) = map(list(i,1), list(i,2));
    end
    [~, index] = sort(valueList, 'descend');
    for i = 1 :size(list,1)
        heuristic(i,:) = [list(index(i),1), list(index(i),2)];
    end

end

% dfs path finding with minimum stack
function path = dfs(map, qs, qg)
    % Init the stack and the visited vector
    row = qs(1); 
    col = qs(2);
    stack = [row, col];
    visited = [];
    
    %until we reach the goal
    while (row ~= qg(1) || col ~= qg(1)) 
        % pop
        row = stack(end, 1);
        col = stack(end, 2);
        stack(end,:) = [];
        % marking the point as visited
        visited = [visited; [row, col]];
        % searching the 8 adjacent cells with min value
        minIndex = minNeighbors(map, row, col);
        % for each valid cell update the stack
        for i = 1 : size(minIndex,1)
            if (~ismember(visited(:,1),minIndex(i,1))) | (~ismember(visited(:,2),minIndex(i,2)))
               stack = [stack; [minIndex(i,1), minIndex(i,2)]];
            end
        end
    end
    path = visited;
end

% minimum finder function
function [minIndex, list] = minNeighbors(map, row, col)

    list = [];
    value = [];
    % 8 adjacent cells
    if row + 1 <= size(map,1) && map(row + 1, col) ~= -1
        list = [list; [row + 1, col]];
        value = [value; map(row + 1, col)];
    end
    if row - 1 >= 1 && map(row - 1, col) ~= -1
        list = [list; [row - 1, col]];
        value = [value; map(row - 1, col)];
    end
    if col + 1 <= size(map,2) && map(row, col + 1) ~= -1
        list = [list; [row, col + 1]];
        value = [value; map(row, col + 1)];
    end
    if col - 1 >= 1 && map(row, col - 1) ~= -1
        list = [list; [row, col - 1]];
        value = [value; map(row, col - 1)];
    end
    if row - 1 >= 1 && col - 1 >= 1 && map(row - 1, col - 1) ~= -1
        list = [list; [row - 1, col - 1]];
        value = [value; map(row - 1, col - 1)];
    end
    if row + 1 <= size(map,1) && col - 1 >= 1 && map(row + 1, col - 1) ~= -1
        list = [list; [row + 1, col - 1]];
        value = [value; map(row + 1, col - 1)];
    end
    if row - 1 >= 1 && col + 1 <= size(map,2) && map(row - 1, col + 1) ~= -1
        list = [list; [row - 1, col + 1]];
        value = [value; map(row - 1, col + 1)];
    end
    if row + 1 <= size(map,1) && col + 1 <= size(map,2) && map(row + 1, col + 1) ~= -1
        list = [list; [row + 1, col + 1]];
        value = [value; map(row + 1, col + 1)];
    end
    % Pick the min value 
    [~, index] = min(value);
    minIndex = list(index,:);

end

function [hop, cost] = hopCounter(map, path)
    % Counting the numer of cell traveled (like the number of router!)
    hop = size(path,1);
    
    % Total cost of the path
    cost = 0;
    for i = 1 : size(path,1)
        cost = cost + map(path(i,1), path(i,2));
    end
end

function list = fourNeighbors(map, row, col)

    list = [];
    % position of south cell
    if row + 1 <= size(map,1) && map(row + 1, col) ~= -1
        list = [list; [row + 1, col]];
    end
    % position of north cell
    if row - 1 >= 1 && map(row - 1, col) ~= -1
        list = [list; [row - 1, col]];
    end
    % position of east cell
    if col + 1 <= size(map,2) && map(row, col + 1) ~= -1
        list = [list; [row, col + 1]];
    end
    % position of west cell
    if col - 1 >= 1 && map(row, col - 1) ~= -1
        list = [list; [row, col - 1]];
    end

end

function list = eightNeighbors(map, row, col)

    list = [];
    if row + 1 <= size(map,1) && map(row + 1, col) ~= -1
        list = [list; [row + 1, col]];
    end
    if row - 1 >= 1 && map(row - 1, col) ~= -1
        list = [list; [row - 1, col]];
    end
    if col + 1 <= size(map,2) && map(row, col + 1) ~= -1
        list = [list; [row, col + 1]];
    end
    if col - 1 >= 1 && map(row, col - 1) ~= -1
        list = [list; [row, col - 1]];
    end
    if row - 1 >= 1 && col - 1 >= 1 && map(row - 1, col - 1) ~= -1
        list = [list; [row - 1, col - 1]];
    end
    if row + 1 <= size(map,1) && col - 1 >= 1 && map(row + 1, col - 1) ~= -1
        list = [list; [row + 1, col - 1]];
    end
    if row - 1 >= 1 && col + 1 <= size(map,2) && map(row - 1, col + 1) ~= -1
        list = [list; [row - 1, col + 1]];
    end
    if row + 1 <= size(map,1) && col + 1 <= size(map,2) && map(row + 1, col + 1) ~= -1
        list = [list; [row + 1, col + 1]];
    end

end
