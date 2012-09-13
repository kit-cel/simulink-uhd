function config()
% CONFIG dependency paths for simulink-uhd build

%% User config area (BEGIN)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (ispc)
    % UHD install directory
    UHD_DIRECTORY = 'C:\Progra~1\UHD\';
    % Boost include directory
    BOOST_DIRECTORY = 'C:\boost\';
    
%% Linux
elseif (isunix) 
    % Leave these empty if boost and uhd are already in your path.
    
    % UHD install directory
    UHD_DIRECTORY = '';
    % Boost include directory
    BOOST_DIRECTORY = '';
    
else
    error('\nYour platform is currently not supported by Simulink-UHD')
end  
% User config area (END)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Build and save settings struct
settings = struct( ...
    'uhd', struct('libdir', fullfile(UHD_DIRECTORY,'lib'), ...
                  'incdir', fullfile(UHD_DIRECTORY,'include')), ...
	'boost', struct('incdir', BOOST_DIRECTORY) ...
);

% mat file to store include and lib paths
settings_file = fullfile(fileparts(mfilename('fullpath')),'settings.mat');
save(settings_file,'settings');
