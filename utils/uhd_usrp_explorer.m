%
% Copyright 2012 Communications Engineering Lab, KIT
%
% This is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 3, or (at your option)
% any later version.
%
% This software is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this software; see the file COPYING. If not, write to
% the Free Software Foundation, Inc., 51 Franklin Street,
% Boston, MA 02110-1301, USA.
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Show graphical USRP explorer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function uhd_usrp_explorer(identifier,varargin)

import javax.swing.*
import javax.swing.tree.*;

% Check input arguments
if (nargin == 0 || nargin > 2 || ~ischar(identifier))
fprintf(['\nUsage: uhd_usrp_explorer(identifier,sensorview) \n\n',...
                        '       This function shows a graphical UHD device explorer.\n\n',...
                        '       identifier: A string that specifies the unique identifier of the UHD device.\n',...
                        '       sensorview: A boolean value that enables the sensor view mode.\n\n',...
                        '       Possible identifiers are:\n\n',...
                        '\t     IP address (addr), e.g. addr=192.168.55.3\n',...
                        '\t     Serial number (serial), e.g. serial=EGR18WFRP\n',...
                        '\t     Device Type (type), e.g. type=usrp2\n',...
                        '\t     Device Name (name), e.g. name=myUSRP\n\n',...
                        '       In the case of an IP Address, the hint addr= can be omitted.\n\n',...
                        'Example: uhd_usrp_explorer(''serial=EGR18WFRP''), or\n',...
                        '         uhd_usrp_explorer(''192.168.55.3'',1)\n\n'])
                    return;
end
                
if ~(uhd_check_device_exists(identifier))
    fprintf('Failed!\n'); 
    msgbox(['UHD Device ' identifier ' does not exist!'],'Warning','warn');
    return;
end
     
treelist = uhd_get_tree(identifier);

if isempty(treelist)
    fprintf('Failed!\n');
    return
end

figname = 'Simulink-UHD USRP Explorer';

if (nargin > 1)
    figname = [figname ' - Sensor View'];
    treelist = wrapper_filter_sensors(treelist);
end

% figure window
f = figure('Name',figname,'MenuBar','none','NumberTitle','off','Units', 'normalized');

% load icons
iconUSRP = fullfile(uhd_get_path('utils'),'resources','usrp.gif');
iconNode = fullfile(uhd_get_path('utils'),'resources','node.gif');
iconLeaf = fullfile(uhd_get_path('utils'),'resources','leaf.gif');

% create top node
cur_node = uitreenode('v0','root', 'USRP', iconUSRP, 0);

% set treeModel
treeModel = DefaultTreeModel( cur_node );

% create the tree
tree = uitree('v0');
tree.setModel( treeModel );
jtree = handle(tree.getTree,'CallbackProperties');

% some layout
drawnow;
set(tree, 'Units', 'normalized', 'position', [0 0.15 0.5 0.85]);

% make root the initially selected node
tree.setSelectedNode( cur_node );

cur_path = '/root';
tree_depth = 1;

%% Create tree

for idx=1:length(treelist)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % example:
    % if key='\mboards\0\eeprom\rev' then 
    %   
    %    base = '\root\mboards\0\eeprom
    %    basevec = {'root' 'mboards' 'eeprom'}
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Get key-value pair from input
    key=treelist{idx,1};
    val=treelist{idx,2};

    % generate base and base_vec
    [base leaf] = fileparts(key); base(1) = '/';
    % more or less a "MATLAB-thing"
    base_vec = regexp(base,'/','split');
    if isempty(base_vec{2}) base_vec = {'root'}; else base_vec(1) = {'root'}; end
    base = full_path(base_vec);

    % get depth of base 
    base_depth = length(base_vec);

    % as long as current tree path is unlike the base path, do ...
    while ~(strcmp(cur_path,base))

        % go up in tree if the depth of the base is smaller than the depth
        % of the current path or the roots of the base path and the current
        % path are different
        if (base_depth < tree_depth || ~strcmp(cur_path,cat_path(base_vec,1:tree_depth)))

            % go one node up
            cur_path = fileparts(cur_path);
            cur_node = cur_node.getParent;
            tree_depth = tree_depth - 1;

        % create new node if the depth of the base is greater than the
        % current path
        else

            path_addon = base_vec{tree_depth + 1};
            childNode = uitreenode('v0', path_addon, path_addon, iconNode, 0);
            treeModel.insertNodeInto(childNode,cur_node,cur_node.getChildCount());
            cur_node = childNode;
            cur_path = [cur_path '/' path_addon];
            tree_depth = tree_depth + 1;

        end
    end

    % create leaf
    childNode = uitreenode('v0', leaf, leaf, iconLeaf, 1);
    treeModel.insertNodeInto(childNode,cur_node,cur_node.getChildCount());
end

% Create listbox
listbox_handle = uicontrol('Style', 'listbox','Units','normalized', 'position', [0.5 0.15 0.5 0.85],'FontWeight','bold');

% Create pushbutton
bushbutton_handle = uicontrol('Style','pushbutton','Units','normalized', 'position', [0.725 0.02 0.25 0.1],'FontWeight','bold','String','Quit','Callback',@pushbuttonpressed);
function pushbuttonpressed(src,eventData) 

    close;
end

%% Helper functions

% MousePressedCallback is not supported by the uitree, but by jtree
set(jtree, 'MousePressedCallback', @mousePressedCallback);
function mousePressedCallback(hTree, eventData) 

    clickX = eventData.getX;
    clickY = eventData.getY;
    treePath = jtree.getPathForLocation(clickX, clickY);
    if ~(isempty(treePath))
        node = treePath.getLastPathComponent;
        value=get_value(node2path(node));
        set(listbox_handle,'String',value);
    end
end

% get value of key
function value = get_value(key)

    temp = regexp(key,'USRP','split');
    key  = temp{2};

    for l=1:length(treelist)

        if (strcmp(treelist{l,1},key))
            value = treelist{l,2};
            break
        else
            value = '';
        end
    end
end

% concatenate parts of a path to single string
function path = cat_path(path_vec,dim)

    path = [];
    for l=dim
        path = [path '/' path_vec{l}];
    end
end

function path = full_path(path_vec)

    path = [];
    for l=1:length(path_vec)
        path = [path '/' path_vec{l}];
    end
end

% get path of node
function path = node2path(node)

    path = node.getPath;
    for i=1:length(path);
        p{i} = char(path(i).getName);
    end
    if length(p) > 1
        path = strrep(fullfile(p{:}),'\','/');
    else
        path = p{1};
    end
end
 
end