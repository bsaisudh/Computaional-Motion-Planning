function [route,numExpanded] = GrassFire (input_map, start_coords, dest_coords)
% Run Grassfire algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.


% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
	0.5 0.5 0.5];

colormap(cmap);

% variable to control if the map is being visualized on every
% iteration
drawMapEveryTime = true;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distanceFromStart = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distanceFromStart(start_node) = 0;

% keep track of number of nodes expanded 
numExpanded = 0;
% Main Loop

visited = [dest_node];
dist = 0;
distance = Inf(nrows,ncols);
% imagecnt = 0;
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    % make drawMapEveryTime = true if you want to see how the 
    % nodes are expanded on the grid. 
    if (drawMapEveryTime)
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
        % name = strcat('result/test_2_',num2str(imagecnt),'.png');
        % saveas(image(1.5, 1.5, map), name);
        % imagecnt = imagecnt + 1;
    end
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:));
    
    % if ((current == dest_node) || isinf(min_dist))
    %     break;
    % end;
    
    % Update map
    % map(current) = 3;         % mark current node as visited
    distanceFromStart(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distanceFromStart), current);
%%    
   % ********************************************************************* 
    % YOUR CODE BETWEEN THESE LINES OF STARS
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
    for i = visited
            current = i;
        % currentxy = ind2sub(size(map), current);
        map(current) = 3;
        distance(current) = dist;
    end
    [toVisit cont] = getneighbours(map, visited);
    for i = toVisit
        current = i;
        % currentxy = ind2sub(size(map), current);
        map(current) = 4;
    end
    visited = toVisit;
    if ~cont
        break;
    end
    dist = dist + 1;
    pause(0.1);
    %*********************************************************************
end

%% Construct route from start to dest by following the parent links
% if (isinf(distanceFromStart(dest_node)))
%     route = [];
% else
    emptyMap = ones(size(map,1)); 
    route = [dest_node];
    current = start_node;
    while true
        [toVisit cont] = getneighbours(emptyMap, current);
        for i = 1:size(toVisit,2)
            dist(i) = distance(toVisit(i));
        end
        [mind , ind] = min(dist);
        route = [route toVisit(ind)];
        current = toVisit(ind);
        if mind == 0
            break;
        end
    end
    
    % while (parent(route(1)) ~= 0)
    %     route = [parent(route(1)), route];
    %  end
    
        % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
        % name = strcat('result/test__',num2str(k),'.png');
        % saveas(image(1.5, 1.5, map), name);
    end
end

% end
%% Get Neighbours

function [toVisit cont] = getneighbours(map, curr)
    toVisit = [];
    mapMin = 1;
    mapMax = size(map,1);
    for current = curr
        [i, j] = ind2sub(size(map), current);
        neighbours = [  i-1 j   ;...
                        i+1 j   ;...
                        i   j+1 ;...
                        i   j-1  ...
                        ];
        for ii = 1:size(neighbours,1)
            if(neighbours(ii,1) >= mapMin & neighbours(ii,1) <= mapMax &... % inside map?
                    neighbours(ii,2) >= mapMin & neighbours(ii,2) <= mapMax &... % inside map?
                    map(neighbours(ii,1), neighbours(ii,2)) == 1 ... % clear cell
                    )
                visit = sub2ind(size(map),neighbours(ii,1),neighbours(ii,2));                    cont = 1;
                toVisit = [toVisit,visit];
            end
        end
    end
    toVisit = unique(toVisit);
    if size(toVisit) > 0
        cont = 1;
    else
        cont = 0;
    end
end
