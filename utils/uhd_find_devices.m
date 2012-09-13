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
% Find USRPs attached to the host
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function uhd_find_devices()


% Check input arguments
narginchk(0, 0);

devices=uhd_find_devices_raw;

if (isempty(devices))
    fprintf('\nNo supported devices found.\n');
    return
end

fprintf('\nFound %d device(s):\n',length(devices))

fprintf('\n%s\t%s\t%s\n','Identifier   ','Type','Name');
fprintf('==============================\n\n');

cnt_dev = 0;
for device=devices
    
    devicename=device.name;
    if (isempty(devicename))
        devicename='<noname>';
    end
    
    identifier=device.addr;
    if isempty(identifier)
        identifier=['serial=' device.serial];
    end
    
    fprintf('%s\t%s\t%s\n',...
        identifier, ...
        device.type, ...
        devicename ...
    );
end

fprintf('\n\n');