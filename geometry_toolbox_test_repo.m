% Gesamttest für die Matlab-Geometrie-Toolbox
% 
% Führt alle verfügbaren Modultests aus um die Funktionalität
% sicherzustellen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-03
% (C) Institut für mechatronische Systeme, Universität Hannover

this_repo_path = fullfile(fileparts(which('geometry_toolbox_path_init.m')));
addpath(fullfile(this_repo_path, 'examples_tests'));

test_box_random_surface_point; close all
test_cylinder_random_surface_point; close all
test_transform_discs; close all
fprintf('Folgende Funktionen erfolgen die manuelle Bestätigung.\n');

test_intersection_box; close all
test_intersection_functions; close all

test_collision_capsule_capsule; close all
test_collision_capsule_sphere; close all
test_collision_sphere_sphere; close all

test_angle_range

clc
close all
fprintf('Alle Testfunktionen dieses Repos ausgeführt\n');