disp('  Adding CDT code to path...   ');
pb_parsers_path = fullfile(fileparts(mfilename('fullpath')));
addpath(genpath(pb_parsers_path));
javaaddpath([pb_parsers_path '/utils/lib/datatypes_java.jar']);
javaaddpath([pb_parsers_path '/utils/lib/protobuf-java.jar']);

% --> CHANGE THESE PATHS TO YOUR USER
addpath('/Users/shcen/code/mex-moos/src')
addpath('/Users/shcen/code/mex-moos/build')

