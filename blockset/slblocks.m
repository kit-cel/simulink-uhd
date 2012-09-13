function blkStruct = slblocks
% Returns information about the Simulink-UHD 
% blockset to the Simulink library browser.
%
% For help on this file, see "Simulink -> Working with Block Libraries ->
% Adding libraries to the Library Browser"

% Information for the "Blocksets and Toolboxes" subsystem (findblib)
blkStruct.Name = sprintf('Simulink-UHD');
blkStruct.OpenFcn = 'simulink_uhd';
blkStruct.MaskDisplay = 'disp(''Simulink-UHD'')';

% Information for the "Simulink Library Browser"
Browser(1).Library = 'simulink_uhd';
Browser(1).Name    = 'Simulink-UHD';
Browser(1).IsFlat  = 0;

blkStruct.Browser = Browser;
clear Browser;

blkStruct.ModelUpdaterMethods.fhUpdateModel = @UpdateSimulinkBlocksHelper;


