% Teste Generierung zufälliger Punkte auf eine Quaderoberfläche

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

clc
close all
clear

%% Definiere zufälligen Quader
R_0_i = eulxyz2r(rand(3,1) );
r_0_Q1 = R_0_i*[0.2 0.2 0.2]';
r_0_Q1Q2 = R_0_i*[0.4 0 0]';% [0.4 0.3 0]';
r_0_Q1Q3 = R_0_i*[0 0.3 0]'; % [0.1 0.5 0.2]'
r_0_Q1Q4 = R_0_i*[0 0 1.6]'; % [-0.1 0.2 0.6]'

r_0_Q2 = r_0_Q1 + r_0_Q1Q2;
r_0_Q3 = r_0_Q1 + r_0_Q1Q3;
r_0_Q4 = r_0_Q1 + r_0_Q1Q4;

Par_j = [r_0_Q1', r_0_Q1Q2', r_0_Q1Q3', r_0_Q1Q4'];

%% Zufällige Punkte auf einer Seite oder auf gesamter Fläche

for jj = 1:4
  
  figure(jj);clf;
  clf; 
  hold on; grid on;axis equal;view(3);set(jj, 'Renderer','OpenGL');
  plot_cube2(r_0_Q1, r_0_Q1Q2, r_0_Q1Q3, r_0_Q1Q4, 'b');

  if jj < 4
    for i = 1:100
      p_i = box_random_surface_point(Par_j, uint8(jj));
      plot3(p_i(1), p_i(2), p_i(3), 'rx');
    end
    title(sprintf('Seite %d', jj));
  else
    for i = 1:1000
      p_i = box_random_surface_point_equal(Par_j);
      plot3(p_i(1), p_i(2), p_i(3), 'rx');
    end
    title('Gleichmäßig verteilt auf gesamter Fläche');
  end

  text(r_0_Q1(1), r_0_Q1(2), r_0_Q1(3), 'Q1');
  text(r_0_Q2(1), r_0_Q2(2), r_0_Q2(3), 'Q2');
  text(r_0_Q3(1), r_0_Q3(2), r_0_Q3(3), 'Q3');
  text(r_0_Q4(1), r_0_Q4(2), r_0_Q4(3), 'Q4');

end