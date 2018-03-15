% Pfad-Initialisierung für die Matlab-Geometrie-Toolbox

% Moritz Schappler, schappler@imes.uni-hannover.de, 2018-03
% (C) Institut für mechatronische Systeme, Universität Hannover

this_tb_path = fileparts( mfilename('fullpath') );
addpath(this_tb_path);

addpath(fullfile(this_tb_path, 'intersection'));
addpath(fullfile(this_tb_path, 'plot'));
addpath(fullfile(this_tb_path, 'surface_points'));
