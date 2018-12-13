% Teste Generierung zufälliger Punkte auf eine Zylinderfläche

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover


clc
close all
clear

%% Definiere zufälligen Zylinder
R_0_i = eulxyz2r(rand(3,1) );
r_0_P1 = R_0_i*[0 0 0.2]';
r_0_P2 = R_0_i*[0 0 0.4]';% [0.4 0.3 0]';
r_zyl = 0.3;

Par_j = [r_0_P1', r_0_P2', r_zyl];

%% Zufällige Punkte auf einer Seite oder auf gesamter Fläche

for jj = 1:4
  figure(jj);clf;
  clf; 
  hold on; grid on;axis equal;view(3);set(jj, 'Renderer','OpenGL');
  drawCylinder([r_0_P1', r_0_P2', r_zyl], 'EdgeColor', 'k', ...
    'FaceAlpha', 0.3, 'FaceColor', 'b')

  if jj < 4
    for i = 1:500
      p_i = cylinder_random_surface_point(Par_j, uint8(jj-1));
      p_0 = R_0_i * p_i;
      plot3(p_i(1), p_i(2), p_i(3), 'rx');
    end
    title(sprintf('Seite %d', jj));
  else
    for i = 1:500
      p_i = cylinder_random_surface_point_equal(Par_j);
      p_0 = R_0_i * p_i;
      plot3(p_i(1), p_i(2), p_i(3), 'rx');
    end

    title('Gleichmäßig verteilt auf gesamter Fläche');
  end
end