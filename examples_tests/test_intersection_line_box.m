% Teste Funktionen für Kollisionsprüfung von Quader und Gerade

% Jonathan Vorndamme, jonathan.vorndamme@tum.de, 2021-05
% (C) Lehrstuhl für Robotik und Systemintelligenz, TU Muenchen

clc
clear
close all;
rng(0);

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
% matlabfcn2mex({'find_intersection_line_box'});
%% Teste Kollision aus Zylinder und Gerade
for i = 1:47 % Schleife über manuelle Testszenarien
  testtol = 1e-10;
  mextol = 1e-12;
  switch i
    case 1
      ug = [0 0 1]';
      rg = [0.5 1 0]';
      S_groundtruth_0 = [0.5 1 0; 0.5 1 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu 2 Seitenpaaren mit Schnitt';
    case 2
      ug = [0 0 1]';
      rg = [0 1 0]';
      S_groundtruth_0 = [0 1 0; 0 1 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu 2 Seitenpaaren tangential an Seite';
    case 3
      ug = [0 0 1]';
      rg = [0 0 0]';
      S_groundtruth_0 = [0 0 0; 0 0 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu 2 Seitenpaaren tangential an Kante';
    case 4
      ug = [0 0 1]';
      rg = [-1e-11 1 0]';
      S_groundtruth_0 = [0 1 0; 0 1 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu 2 Seitenpaaren fast tangential an Seite';
    case 5
      ug = [0 0 1]';
      rg = [-1e-11 0 0]';
      S_groundtruth_0 = [0 0 0; 0 0 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu 2 Seitenpaaren fast tangential an Kante';
    case 6
      ug = [0 0 1]';
      rg = [0 3 0]';
%       S_groundtruth_0 = [0 2 0; 1 3 NaN]';
      S_groundtruth_0 = [0 2 0; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu 2 Seitenpaaren ohne Schnitt drüber';
    case 7
      ug = [0 0 1]';
      rg = [2 3 0]';
%       S_groundtruth_0 = [1 2 0; sqrt(2) 3 NaN]';
      S_groundtruth_0 = [1 2 0; sqrt(2) NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu 2 Seitenpaaren ohne Schnitt vorbei';
    case 8
      ug = [0 1 4]';
      rg = [0.5 1/4 0]';
      S_groundtruth_0 = [0.5 1/4 0; 0.5 1 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, Schnitt gegenüber liegende Seiten';
    case 9
      ug = [0 1 1]';
      rg = [0.5 1 0]';
      S_groundtruth_0 = [0.5 1 0; 0.5 2 1]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, Schnitt benachbarte Seiten';
    case 10
      ug = [0 1 1]';
      rg = [0.5 0 0]';
      S_groundtruth_0 = [0.5 0 0; 0.5 2 2]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, Schnitt Kante und ggü. Seite';
    case 11
      ug = [0 2 3]';
      rg = [0.5 0 0]';
      S_groundtruth_0 = [0.5 0 0; 0.5 2 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, Schnitt Kante und ggü. Kante';
    case 12
      ug = [0 1 4]';
      rg = [1 1/4 0]';
      S_groundtruth_0 = [1 1/4 0; 1 1 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, tangential an Seite, Schnitt Kante ggü. Kante';
    case 13
      ug = [0 1 1]';
      rg = [1 1 0]';
      S_groundtruth_0 = [1 1 0; 1 2 1]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, tangential an Seite, Schnitt benachbarte Kanten';
    case 14
      ug = [0 1 1]';
      rg = [0 0 0]';
      S_groundtruth_0 = [0 0 0; 0 2 2]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, tangential an Seite, Ecke ggü. Kante';
    case 15
      ug = [0 2 3]';
      rg = [0 0 0]';
      S_groundtruth_0 = [0 0 0; 0 2 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, tangential an Seite, Schnitt Ecke ggü. Ecke';
    case 16
      ug = [0 1 1]';
      rg = [0.5 0 3]';
      S_groundtruth_0 = [0.5 0 3; 0.5 0 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, tangential an Kante';
    case 17
      ug = [0 1 1]';
      rg = [0 0 3]';
      S_groundtruth_0 = [0 0 3; 0 0 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, tangential an Ecke';
    case 18
      ug = [0 1 4]';
      rg = [1+1e-11 1/4 0]';
      S_groundtruth_0 = [1 1/4 0; 1 1 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, fast tangential an Seite, Schnitt Kante ggü. Kante';
    case 19
      ug = [0 1 1]';
      rg = [1+1e-11 1 0]';
      S_groundtruth_0 = [1 1 0; 1 2 1]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, fast tangential an Seite, Schnitt benachbarte Kanten';
    case 20
      ug = [0 1 1]';
      rg = [0-1e-11 0 0]';
      S_groundtruth_0 = [0 0 0; 0 2 2]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, fast tangential an Seite, Ecke ggü. Kante';
    case 21
      ug = [0 2 3]';
      rg = [0-1e-11 0 0]';
      S_groundtruth_0 = [0 0 0; 0 2 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, fast tangential an Seite, Schnitt Ecke ggü. Ecke';
    case 22
      ug = [0 1 1]';
      rg = [0.5 -1e-11 3+1e-11]';
      S_groundtruth_0 = [0.5 0 3; 0.5 0 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, fast tangential an Kante';
    case 23
      ug = [0 1 1]';
      rg = [0 -1e-11 3+1e-11]';
      S_groundtruth_0 = [0 0 3; 0 0 3]';
      intersect_truth = true;
      tt = 'Gerade parallel zu einem Seitenpaar, fast tangential an Ecke';
    case 24
      ug = [0 1 4]';
      rg = [2 1/4 0]';
%       S_groundtruth_0 = [1 1/4 0; 1 3/4*sqrt(17) NaN]';
      S_groundtruth_0 = [1 1/4 0; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu einem Seitenpaar, vorbei, drüber über Kante ggü. Kante';
    case 25
      ug = [0 1 1]';
      rg = [2 1 0]';
%       S_groundtruth_0 = [1 1 0; 1 sqrt(2) NaN]';
      S_groundtruth_0 = [1 1 0; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu einem Seitenpaar, vorbei, drüber über benachbarte Kanten';
    case 26
      ug = [0 1 1]';
      rg = [2 0 0]';
%       S_groundtruth_0 = [1 0 0; 1 2*sqrt(2) NaN]';
      S_groundtruth_0 = [1 0 0; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu einem Seitenpaar, vorbei, drüber über Ecke, ggü. Kante';
    case 27
      ug = [0 2 3]';
      rg = [2 0 0]';
%       S_groundtruth_0 = [1 0 0; 1 sqrt(13) NaN]';
      S_groundtruth_0 = [1 0 0; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu einem Seitenpaar, vorbei, drüber über ggü. Ecken';
    case 28
      ug = [0 1 1]';
      rg = [2 2 3]';
      S_groundtruth_0 = [1 2 3; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu einem Seitenpaar, vorbei, tangential an Kantenverlängerung';
    case 29
      ug = [0 1 1]';
      rg = [2 2+1e-11 3+1e-11]';
      S_groundtruth_0 = [1 2 3; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu einem Seitenpaar, vorbei, fast tangential an Kantenverlängerung';
    case 30
      ug = [1 0 1]';
      rg = [2 3 3]';
      S_groundtruth_0 = [0 2 1; 1 NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade parallel zu einem Seitenpaar, aussen vorbei';
    case 31
      ug = [1 1 1]';
      rg = [0 0.5 1]';
      S_groundtruth_0 = [0 0.5 1; 1 1.5 2]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt ggü. Seiten';
    case 32
      ug = [1 1 1]';
      rg = [0 1.5 1]';
      S_groundtruth_0 = [0 1.5 1; 0.5 2 1.5]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt benachbarte Seiten';
    case 33
      ug = [1 1 1]';
      rg = [0 1 1]';
      S_groundtruth_0 = [0 1 1; 1 2 2]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt Seite ggü. Kante';
    case 34
      ug = [1 1 1]';
      rg = [0 1.5 2.5]';
      S_groundtruth_0 = [0 1.5 2.5; 0.5 2 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt Seite benachbarte Kante';
    case 35
      ug = [1 1 1]';
      rg = [0 1 2]';
      S_groundtruth_0 = [0 1 2; 1 2 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt Seite ggü. Ecke';
    case 36
      ug = [1 2 1]';
      rg = [0 0 1]';
      S_groundtruth_0 = [0 0 1; 1 2 2]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt ggü. Kanten';
    case 37
      ug = [1 1 1]';
      rg = [0 0 2]';
      S_groundtruth_0 = [0 0 2; 1 1 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt diagonale Kanten';
    case 38
      ug = [1 2 1]';
      rg = [0 0 2]';
      S_groundtruth_0 = [0 0 2; 1 2 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt Kante ggü. Ecke';
    case 39
      ug = [1 2 3]';
      rg = [0 0 0]';
      S_groundtruth_0 = [0 0 0; 1 2 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, Schnitt ggü. Ecken';
    case 40
      ug = [1 1 1]';
      rg = [0 1 3]';
      S_groundtruth_0 = [0 1 3; 0 1 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, tangential an Kante';
    case 41
      ug = [1 1 1]';
      rg = [0 0 3]';
      S_groundtruth_0 = [0 0 3; 0 0 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, tangential an Ecke';
    case 42
      ug = [1 1 1]';
      rg = [0 1+1e-11 3+1e-11]';
      S_groundtruth_0 = [0 1 3; 0 1 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, fast tangential an Kante';
    case 43
      ug = [1 1 1]';
      rg = [0 0 3+1e-11]';
      S_groundtruth_0 = [0 0 3; 0 0 3]';
      intersect_truth = true;
      tt = 'Gerade windschief, fast tangential an Ecke';
    case 44
      ug = [1 1 1]';
      rg = [0 0 4]';
      S_groundtruth_0 = [0 0 3; sqrt(2/3) NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade windschief, vorbei über Fläche';
    case 45
      ug = [1 1 1]';
      rg = [0 2 4]';
      S_groundtruth_0 = [0 1.5 3; 1/sqrt(2) NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade windschief, vorbei Kantenverlängerung tangiert';
    case 46
      ug = [1 1 1]';
      rg = [0 2+1e-11 4]';
      S_groundtruth_0 = [0 1.5 3; 1/sqrt(2) NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade windschief, vorbei Kantenverlängerung fast tangiert';
    case 47
      ug = [1 1 1]';
      rg = [2 5 8.5]';
      S_groundtruth_0 = [0 1.25 3; 0.25*sqrt(98) NaN NaN]';
      intersect_truth = false;
      tt = 'Gerade windschief, aussen vorbei';
  end
  for j=1:2
    if j==2 % change direction of ug
      ug = -ug;
      % Bei umgedrehter Richtung von u wird auch das ergebnis anders herum sein
      if ~isnan(S_groundtruth_0(3,2))
        S_groundtruth_0 = S_groundtruth_0(1:3,[2 1]);
      elseif ~isnan(S_groundtruth_0(2,2))
        S_groundtruth_0(1:3,1) = S_groundtruth_0(1:3,1)-S_groundtruth_0(2,2)*ug/norm(ug);
      end
    end
    q = [0;0;0];
    u = [1 0 0;0 2 0;0 0 3];
    for k=0:7 % use all corners as base
      if mod(k,2)
        q = q + u(:,1);
        u(:,1) = -u(:,1);
      end
      if mod(floor(k/2),2)
        q = q + u(:,2);
        u(:,2) = -u(:,2);
      end
      if mod(floor(floor(k/2)/2),2)
        q = q + u(:,3);
        u(:,3) = -u(:,3);
      end
      for l=perms(1:3)' % use all possible combinations of directions
        for m = 1:3 % zufällige Transformation aller Punkte
          %% Transformation
          if m == 1 % keine Transformation
            T_W_0 = eye(4);
          elseif m == 2 % Zufällige Transformation
            phi = 180*2*(-0.5+rand(3,1)); % [-180,180]
            T_W_0 = eulerAnglesToRotation3d(phi(1),phi(2),phi(3))* ...
                    createTranslation3d(rand(3,1));
          else % Drehung so, dass ein Sonderfall eintritt
            T_W_0 = createRotationOx(-pi/2)*createTranslation3d(rand(3,1));
          end
          q_W = eye(3,4)*T_W_0*[q;1];
          u_W = T_W_0(1:3,1:3)*u;
          if intersect_truth
            Skol_groundtruth_W = (eye(3,4)*T_W_0*[S_groundtruth_0;ones(1,2)]);
          else
            Skol_groundtruth_W(1:3,1) = (eye(3,4)*T_W_0*[S_groundtruth_0(1:3,1);1]);
            Skol_groundtruth_W(1:3,2) = S_groundtruth_0(1:3,2);
          end
          rg_W = eye(3,4)*T_W_0*[rg;1];
          ug_W = T_W_0(1:3,1:3)*ug;
          %% Schnitt berechnen
          S_tmp = find_intersection_line_box(rg_W, ug_W, q_W, u_W(:,1), u_W(:,2), u_W(:,3));
          pt1 = S_tmp(:,1);
          pt2 = S_tmp(:,2);
          if any(isnan(S_tmp(:,2)))
            dist = S_tmp(1,2);
          else
            dist = 0;
          end
          %% Zeichnen
          change_current_figure(i);clf; hold on;
          cubpar_c = q+(u_W(:,1)+u_W(:,2)+u_W(:,3))/2; % Mittelpunkt des Quaders
          cubpar_l = [norm(u_W(:,1)); norm(u_W(:,2)); norm(u_W(:,3))]; % Dimension des Quaders
          cubpar_a = 180/pi*r2eulzyx([u_W(:,1)/norm(u_W(:,1)), u_W(:,2)/norm(u_W(:,2)), u_W(:,3)/norm(u_W(:,3))]); % Orientierung des Quaders
          drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', 'b', 'FaceAlpha', 0.3);
          plot3(pt1(1), pt1(2), pt1(3), 'kx', 'MarkerSize', 10, 'LineWidth', 3);
          plot3(pt2(1), pt2(2), pt2(3), 'r+', 'MarkerSize', 12, 'LineWidth', 3);
          plot3([Skol_groundtruth_W(1,1);S_tmp(1,1)], [Skol_groundtruth_W(2,1);S_tmp(2,1)], ...
                [Skol_groundtruth_W(3,1);S_tmp(3,1)], '--c', 'MarkerSize', 5, 'LineWidth', 3);
          if intersect_truth
            plot3([Skol_groundtruth_W(1,2);S_tmp(1,2)], [Skol_groundtruth_W(2,2);S_tmp(2,2)], ...
                  [Skol_groundtruth_W(3,2);S_tmp(3,2)], '--c', 'MarkerSize', 5, 'LineWidth', 3);
          end
          xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
          view(3); grid on; axis equal;
          title(sprintf('Fall %d: %s. Abstand: %1.2f', i, tt, dist));
          mm = minmax2([pt1';pt2';Skol_groundtruth_W(1:3,1)';Skol_groundtruth_W(1:3,2)';...
                        q_W'; q_W'+u(:,1)'; q_W'+u(:,2)'; q_W'+u(:,3)'; q_W'+u(:,1)'+u(:,2)';...
                        q_W'+u(:,1)'+u(:,3)'; q_W'+u(:,2)'+u(:,3)'; q_W'+u(:,1)'+u(:,2)'+u(:,3)';]');
          lambda_min = (-2 + mm(:,1) - rg_W)./ug_W;
          lambda_max = (2 + mm(:,2) - rg_W)./ug_W;
          lambda = sort([lambda_min;lambda_max]);
          lambda_min = lambda(3);
          lambda_max = lambda(4);
          plot3([rg_W(1)+lambda_min*ug_W(1);rg_W(1)+lambda_max*ug_W(1)], ...
                [rg_W(2)+lambda_min*ug_W(2);rg_W(2)+lambda_max*ug_W(2)], ...
                [rg_W(3)+lambda_min*ug_W(3);rg_W(3)+lambda_max*ug_W(3)], 'b-');
          xlim([-2+mm(1,1), 2+mm(1,2)]);
          ylim([-2+mm(2,1), 2+mm(2,2)]);
          zlim([-2+mm(3,1), 2+mm(3,2)]);
          drawnow();
          %% Prüfung
          % Rechnerische Prüfung der Ergebnisse
          if dist == 0 && ~intersect_truth
            error('Schnittpunkt nicht manuell gesetzt, aber berechnet');
          end
          if dist ~= 0 && intersect_truth
            error('Schnittpunkt manuell gesetzt, aber nicht berechnet');
          end
          if intersect_truth
            % Falls händisch Schnittpunkte bestimmt wurden
            assert(all(abs(S_tmp - Skol_groundtruth_W) < testtol, [1 2]), ...
              'Händisch bestimmter Schnittpunkt stimmt nicht mit Berechnung überein: \n[%s]\n vs \n[%s]\nAbweichung : %e', ...
              disp_array(Skol_groundtruth_W, '%1.3f'), disp_array(S_tmp, '%1.3f'), ...
              max(abs(S_tmp - Skol_groundtruth_W),[],'all'));
          else
            % Falls händisch der nächste Punkt bestimmt wurde
            ind = ~isnan(S_groundtruth_0);
            % take into account that for different direction of u, the expecteds
            % result changes if u is parallel to cylinder axis or sides
            assert(all(abs(S_tmp(ind) - Skol_groundtruth_W(ind)) < testtol, [1 2]), ...
              'Händisch bestimmter nächster Punkt stimmt nicht mit Berechnung überein: \n[%s]\n vs \n[%s]\nAbweichung : %e', ...
              disp_array(Skol_groundtruth_W, '%1.3f'), disp_array(S_tmp, '%1.3f'), ...
              min(max(abs(S_tmp(ind) - Skol_groundtruth_W(ind)),[],'all')));
          end
          % Prüfung gegen Ausgabe der mex-Funktion
          S_tmp2 = find_intersection_line_box_mex(rg_W, ug_W, q_W, u_W(:,1), u_W(:,2), u_W(:,3));
          % prüfe auch für getauschte Punkte, da mex-Funktion Ergebnisse in
          % mindestens einem Fall i umgekehter Reihenfolge lieferte
          assert(all(abs(S_tmp(~isnan(S_tmp(:))) - S_tmp2(~isnan(S_tmp(:)))) < mextol), ...
            'Ausgabevariable S_tmp stimmt nicht mit mex-Funktion überein:\n[%s] vs \n[%s]\nAbweichung: %e', ...
            disp_array(S_tmp, '%1.3f'), disp_array(S_tmp2, '%1.3f'), ...
            min(max(abs(S_tmp(~isnan(S_tmp(:))) - S_tmp2(~isnan(S_tmp(:)))),[],'all')));
        end
      end
    end
  end
end
fprintf('Testfälle für Kollision Quader und Gerade erfolgreich absolviert.\n');