% Teste Funktionen für Kollisionsprüfung von Zylinder und Gerade

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all;
rng(0);

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({'find_intersection_line_capsule'});
%% Teste Kollision aus Zylinder und Gerade
for j = 1:4 % Schleife über Vertauschung der Enden und Richtungsänderung der Geraden
  % Vertausche die Reihenfolge der Punkte, um mehr Code in Test abzudecken.
  % Darf keinen Einfluss auf Ergebnis haben
  if j == 1
    cap_0 = [0,0,0, 0,0,5, 1];
  elseif j == 3
    cap_0 = [0,0,5, 0,0,0, 1];
  end
  for i = 1:21 % Schleife über manuelle Testszenarien
    testtol = 1e-7;
    mextol = 1e-7;
    switch i
      case 1
        ug = [0 0 1]';
        rg = [0 0 6]';
        S_groundtruth_0 = [0 0 -1; 0 0 6]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Mittellinie mit Schnitt';
      case 2
        ug = [0 0 1]';
        rg = [1 0 6]';
        S_groundtruth_0 = [1 0 0; 1 0 5]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Mittellinie tangential';
      case 3
        ug = [0 0 1]';
        rg = [1+1e-11 0 6]';
        S_groundtruth_0 = [1 0 0; 1 0 5]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Mittellinie fast tangential';
      case 4
        ug = [0 0 1]';
        rg = [2 0 6]';
        S_groundtruth_0 = [1 0 0; 1 5 NaN]';
        intersect_truth = false;
        tt = 'Gerade parallel zu Mittellinie ohne Schnitt';
      case 5
        ug = [1 0 1]';
        rg = [0 0 2]';
        S_groundtruth_0 = [-1 0 1; 1 0 3]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt 2x Mantel';
      case 6
        ug = [1 0 1]';
        rg = [0 0 5]';
        S_groundtruth_0 = [-1 0 4; sqrt(2)/2 0 5+sqrt(2)/2]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt Mantel und Kuppel';
      case 7
        ug = [sqrt(2) 0 5+sqrt(2)]';
        rg = [-sqrt(2)/2 0 -sqrt(2)/2]';
        S_groundtruth_0 = [-sqrt(2)/2 0 -sqrt(2)/2; sqrt(2)/2 0 5+sqrt(2)/2]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt beide Kuppeln';
      case 8
        ug = [1 0 0]';
        rg = [0 0 5+sqrt(2)/2]';
        S_groundtruth_0 = [-sqrt(2)/2 0 5+sqrt(2)/2; sqrt(2)/2 0 5+sqrt(2)/2]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt 2x Kuppel';
      case 9
        ug = [1 0 1]';
        rg = [1 0 5]';
        S_groundtruth_0 = [-1 0 3; 1 0 5]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt Übergang und Mantel';
      case 10
        ug = [1 0 6]';
        rg = [0 0 -1]';
        S_groundtruth_0 = [0 0 -1; 1 0 5]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt Übergang und Kuppel';
      case 11
        ug = [1 0 5/2]';
        rg = [1 0 5]';
        S_groundtruth_0 = [-1 0 0; 1 0 5]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt 2x Übergang';
      case 12
        ug = [1 0 1]';
        rg = [0 1 2]';
        S_groundtruth_0 = [0 1 2; 0 1 2]';
        intersect_truth = true;
        tt = 'Gerade windschief tangential am Mantel';
      case 13
        ug = [1 0 1]';
        rg = [0 1 5]';
        S_groundtruth_0 = [0 1 5; 0 1 5]';
        intersect_truth = true;
        tt = 'Gerade windschief tangential am Übergang';
      case 14
        ug = [1 0 1]';
        rg = [0 1 7]';
        S_groundtruth_0 = [-1/sqrt(3) 1/sqrt(3) 5+1/sqrt(3); sqrt(3)-1 NaN NaN]';
        intersect_truth = false;
        tt = 'Gerade windschief tangential am Mantel ausserhalb';
      case 15
        ug = [1 0 1]';
        rg = [0 1+1e-11 2]';
        S_groundtruth_0 = [0 1 2; 0 1 2]';
        intersect_truth = true;
        tt = 'Gerade windschief fast tangential am Mantel';
      case 16
        ug = [1 0 1]';
        rg = [0 1+1e-11 5]';
        S_groundtruth_0 = [0 1 5; 0 1 5]';
        intersect_truth = true;
        tt = 'Gerade windschief fast tangential am Übergang';
      case 17
        ug = [1 0 1]';
        rg = [0 1+1e-11 7]';
        S_groundtruth_0 = [-1/sqrt(3) 1/sqrt(3) 5+1/sqrt(3); sqrt(3)-1 NaN NaN]';
        intersect_truth = false;
        tt = 'Gerade windschief fast tangential am Mantel ausserhalb';
      case 18
        ug = [1 0 1]';
        rg = [0 2 2]';
        S_groundtruth_0 = [0 1 2; 1 NaN NaN]';
        intersect_truth = false;
        tt = 'Gerade windschief mittig vorbei';
      case 19
        ug = [1 0 1]';
        rg = [0 0 7]';
        S_groundtruth_0 = [-sqrt(2)/2 0 5+sqrt(2)/2; sqrt(2)-1 NaN NaN]';
        intersect_truth = false;
        tt = 'Gerade windschief jenseits Kuppel, Abstand Zylinderachse < r';
      case 20
        ug = [1 0 1]';
        rg = [0 2 7]';
        % gefunden mit brute force (Abstand Punkt Gerade für gesamten Kreis
        % berechnet)
        S_groundtruth_0 = [-1/sqrt(6) 2/sqrt(6) 5+1/sqrt(6); sqrt(6)-1 NaN NaN]';
        intersect_truth = false;
        tt = 'Gerade windschief jenseits Kuppel, Abstand Zylinderachse > r';
      case 21
        ug = [-1e-12 0 1]';
        rg = [1 0 2.5]';
        S_groundtruth_0 = [1 0 0; 1 0 5]';
        intersect_truth = true;
        tt = 'Gerade fast parallel zu Mittellinie tangential';
    end
    if j == 2 || j == 4
      ug = -ug;
      % Bei umgedrehter Richtung von u wird auch das ergebnis anders herum sein
      if ~isnan(S_groundtruth_0(3,2))
        S_groundtruth_0 = S_groundtruth_0(1:3,[2 1]);
      elseif ~isnan(S_groundtruth_0(2,2))
        S_groundtruth_0(1:3,1) = S_groundtruth_0(1:3,1)-S_groundtruth_0(2,2)*ug/norm(ug);
      end
    end
    for k = 1:3 % zufällige Transformation aller Punkte
      %% Transformation
      if k == 1 % keine Transformation
        T_W_0 = eye(4);
      elseif k == 2 % Zufällige Transformation
        phi = 180*2*(-0.5+rand(3,1)); % [-180,180]
        T_W_0 = eulerAnglesToRotation3d(phi(1),phi(2),phi(3))* ...
                createTranslation3d(rand(3,1));
      else % Drehung so, dass ein Sonderfall eintritt
        T_W_0 = createRotationOx(-pi/2)*createTranslation3d(rand(3,1));
      end
      cap_W = [eye(3,4)*T_W_0*[cap_0(1:3)';1]; eye(3,4)*T_W_0*[cap_0(4:6)';1]; cap_0(7)]';
      if intersect_truth
        Skol_groundtruth_W = (eye(3,4)*T_W_0*[S_groundtruth_0;ones(1,2)]);
      else
        Skol_groundtruth_W(1:3,1) = (eye(3,4)*T_W_0*[S_groundtruth_0(1:3,1);1]);
        Skol_groundtruth_W(1:3,2) = S_groundtruth_0(1:3,2);
      end
      rg_W = eye(3,4)*T_W_0*[rg;1];
      ug_W = T_W_0(1:3,1:3)*ug;
      %% Schnitt berechnen
      S_tmp = find_intersection_line_capsule(rg_W, ug_W, cap_W(1:3)', cap_W(4:6)', cap_W(7));
      pt1 = S_tmp(:,1);
      pt2 = S_tmp(:,2);
      if any(isnan(S_tmp(:,2)))
        dist = S_tmp(1,2);
      else
        dist = 0;
      end
      %% Zeichnen
      change_current_figure(i);clf; hold on;
      drawCapsule(cap_W,'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', ':');
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
                    cap_W(1:3)+cap_W(7);cap_W(1:3)-cap_W(7);cap_W(4:6)+cap_W(7);cap_W(4:6)-cap_W(7)]');
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
      S_tmp2 = find_intersection_line_capsule_mex(rg_W, ug_W, cap_W(1:3)', ...
        cap_W(4:6)', cap_W(7));
      % prüfe auch für getauschte Punkte, da mex-Funktion Ergebnisse in
      % mindestens einem Fall i umgekehter Reihenfolge lieferte
      assert(all(abs(S_tmp(~isnan(S_tmp(:))) - S_tmp2(~isnan(S_tmp(:)))) < mextol), ...
        'Ausgabevariable S_tmp stimmt nicht mit mex-Funktion überein:\n[%s] vs \n[%s]\nAbweichung: %e', ...
        disp_array(S_tmp, '%1.3f'), disp_array(S_tmp2, '%1.3f'), ...
        min(max(abs(S_tmp(~isnan(S_tmp(:))) - S_tmp2(~isnan(S_tmp(:)))),[],'all')));
      % Prüfung, ob einer der erkannten Punkte auf der Kapsel liegt
      [dist_cp1, kol_cp1, pkol_cp1, d_min_cp1] = collision_capsule_sphere(cap_W, [pt1' 0]);
      [dist_cp2, kol_cp2, pkol_cp2, d_min_cp2] = collision_capsule_sphere(cap_W, [pt2' 0]);
      if ~any(abs([dist_cp1;dist_cp2]) < 1e-10)
        error('Keiner der erkannten Punkte liegt auf der Kapsel');
      end
    end
  end
end
fprintf('Testfälle für Kollision Kapsel und Gerade erfolgreich absolviert.\n');