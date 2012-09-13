function make(varargin)
% MAKE Script to build the Simulink-UHD blockset
%
% Syntax: MAKE(arg1, arg2, ...) or MAKE arg1 arg2
% Args: -v  verbose mode
%       -g  add debug symbols
%       ... (same as MEX script)
%       -f  force building of all targets (replaces mex -f)
%       -s  auto save source files opened in editor
%
% MAKE reads the include and library paths for UHD and Boost from
% ./settings.mat which can be generated with config.m.
%
% After building remember to add the subfolders bin, blockset, help and
% utils to your MATLAB path environment.
%
% Copyright 2012 Communications Engineering Lab, KIT

build_root = fileparts(mfilename('fullpath'));

%% define targets and their source files
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% setting.mat holds include and lib paths for uhd and boost
settings_file = fullfile(build_root,'settings.mat');

% source files for different modules
target_sources = {
    { 'uhd_sink.cpp'; 'param_setter.cpp'; 'getMxStringArray.cpp' }
    { 'uhd_source.cpp'; 'param_setter.cpp'; 'getMxStringArray.cpp' }
    { 'uhd_sensor.cpp' }
    { 'uhd_find_devices_raw.cpp' }
    { 'uhd_get_tree.cpp' }
};

target_libs = { 'uhd' };

% output directory
bin_path = fullfile(build_root,'bin');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Build configs

% Load boost and uhd paths 
if exist(settings_file,'file')
    load(settings_file);
else
    error('Settings file not found. Edit and run config.m to create one');
end

% Extract custom flags from varargin
save_sources = any(strcmp('-s',varargin)); varargin(strcmp('-s',varargin)) = [];
force_build =  any(strcmp('-f',varargin)); varargin(strcmp('-f',varargin)) = [];

% Add full path to source files 
target_sources = cellfun(@(target) ...
    cellfun(@(x) fullfile(build_root,'src',char(x)),target, 'UniformOutput',false), ...
    target_sources,  'UniformOutput',false ...
);

% Put it all together
configs = struct( ...
    'args', {[varargin,  {'-outdir',bin_path}]'}, ...
    'incpaths', {{fullfile(build_root,'include'); ...
                  settings.uhd.incdir;
                  settings.boost.incdir}}, ...
    'libpaths', {{settings.uhd.libdir}}, ...
    'srcs', target_sources, ...
    'libs', {target_libs});
% NOTE: same args, inc/lib paths for each target, different sources

if ~exist(bin_path,'dir'), mkdir(bin_path); end
clear settings* target_*

%% Build mex arguments from configs & execute mex
build_count = 0;
for config = configs' % for each target
    
    % Invoke 'Save' on modified source files
    if save_sources && exist('matlab.desktop.editor.Document','class')
        for src  = config.srcs'
            doc = matlab.desktop.editor.findOpenDocument(char(src));
            if ~isempty(doc) && doc.Modified, doc.save(); end; 
        end
    end
    
    % Get target info
    target = struct();
    [target.path target.name ~] = fileparts(config.srcs{1});
    target.file = [target.name '.' mexext()];
    
    % Check if existing build in up-to-date
    if ~force_build
        uptodate = true;
        target_info = dir(fullfile(bin_path,target.file));
        for src  = config.srcs' 
            source_info = dir(char(src));
            uptodate = uptodate && ~isempty(target_info) && ...
                target_info.datenum >= source_info.datenum;
        end
        clear target_info source_info
        if uptodate, continue; end % skip this target if up-to-date
    end    
        
    % Build mex args        
    config.incpaths = cellfun(@(x) strcat('-I', char(x)), config.incpaths, 'UniformOutput',false);
    config.libpaths = cellfun(@(x) strcat('-L', char(x)), config.libpaths, 'UniformOutput',false);
    config.srcs     = cellfun(@(x)              char(x) , config.srcs,     'UniformOutput',false);
    config.libs     = cellfun(@(x) strcat('-l', char(x)), config.libs,     'UniformOutput',false);
    mexargc = struct2cell(config);
    mexargc = cat(1, mexargc{:});
    
    % Run mex command
    fprintf('Building %s...', target.file);
    if any(strcmp('-v',varargin)), fprintf('\n'); end
    
    mex(mexargc{:})
    build_count = build_count + 1;
           
    disp('Done!');
    if ~any(strcmp('-v',varargin)), fprintf('\n'); end
end

if build_count
    disp('Successfully build all targets.');
else
    disp('All targets uptodate. Use make -f to force a rebiuld');
end
