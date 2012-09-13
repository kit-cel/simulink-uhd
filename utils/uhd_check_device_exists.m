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
% Query sensors of specific device (identifier)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [attached,device] = uhd_check_device_exists(identifier)

% Check input arguments
if (nargin ~= 1 || ~ischar(identifier))
fprintf(['\nUsage: [attached,device] = uhd_check_device_exists(identifier) \n\n',...
                        '       This function will check if an UHD device is attached to the host.\n\n',...
                        '       identifier: A string that specifies the unique identifier of the UHD device.\n',...
                        '       attached: A boolean value that indicates if the UHD device is attached.\n',...
                        '       device: If the device is attached, this MATLAB struct contains the device information.\n\n', ...
                        '       Possible identifiers are:\n\n',...
                        '\t     IP address (addr), e.g. addr=192.168.55.3\n',...
                        '\t     Serial number (serial), e.g. serial=EGR18WFRP\n',...
                        '\t     Device Type (type), e.g. type=usrp2\n',...
                        '\t     Device Name (name), e.g. name=myUSRP\n\n',...
                        '       In the case of an IP Address, the hint addr= can be omitted.\n\n',...
                        'Example: [a,b]=uhd_check_device_exists(''serial=EGR18WFRP'')\n\n']);
                    return;
end

device=uhd_find_devices_raw(identifier);

if isempty(device)
    attached=0;
    device=[];
else
    attached=1;
end