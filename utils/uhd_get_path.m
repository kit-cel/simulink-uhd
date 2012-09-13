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
% get path of the Simulink-UHD files
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Path] = uhd_get_path(component)

% Check input arguments
narginchk(0, 1);

% Find paths
utilsPath = fileparts(mfilename('fullpath'));
Path=fullfile(fileparts(utilsPath));

if (nargin > 0)
    if strfind(component,'help')
        Path=fullfile(fileparts(utilsPath),'help');
    elseif strfind(component,'blockset')
        Path=fullfile(fileparts(utilsPath),'blockset');
    elseif strfind(component,'bin')
        Path=fullfile(fileparts(utilsPath),'bin');
    elseif strfind(component,'utils')
        Path=utilsPath;
    else 
        Path=fullfile(fileparts(utilsPath));
    end
end
    
end
