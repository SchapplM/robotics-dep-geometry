% Teste Funktionen für Kollisionsprüfung von Zylinder und Gerade
% 
% Ergebnis:
% * Fehler der Funktion find_intersection_line_cylinder in Grenzfall.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
rng(0);

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
% matlabfcn2mex({'find_intersection_line_cylinder'});
%% Teste Kollision aus Zylinder und Gerade
for j = 1:2 % Schleife über Vertauschung der Enden
  % Vertausche die Reihenfolge der Punkte, um mehr Code in Test abzudecken.
  % Darf keinen Einfluss auf Ergebnis haben
  if j == 1
    cyl_0 = [0,0,0, 0,0,5, 1];
  elseif j == 2
    cyl_0 = [0,0,5, 0,0,0, 1];
  end
  for i = 1:20 % Schleife über manuelle Testszenarien
    testtol = 1e-9;
    switch i
      case 1
        ug = [0 0 1]';
        rg = [0 0 6]';
        pt_groundtruth_0 = [0 0 0]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Mittellinie mit Schnitt';
      case 2
        ug = [0 0 1]';
        rg = [2 0 6]';
        pt_groundtruth_0 = [1 0 0]';
        intersect_truth = false;
        tt = 'Gerade parallel zu Mittellinie ohne Schnitt';
      case 3
        ug = [0 0 1]';
        rg = [1 0 6]';
        pt_groundtruth_0 = [1 0 0]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Mittellinie tangential';
      case 4
        ug = [1 0 0]';
        rg = [0 0 2]';
        pt_groundtruth_0 = [-1 0 2]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Deckel mit Schnitt';
      case 5
        ug = [1 0 0]';
        rg = [0 0 6]';
        pt_groundtruth_0 = [-1 0 5]';
        intersect_truth = false;
        tt = 'Gerade parallel zu Deckel aussen vorbei, Abstand Zylinderachse < r';
      case 6
        ug = [1 0 0]';
        rg = [0 2 6]';
        pt_groundtruth_0 = [0 1 5]';
        intersect_truth = false;
        tt = 'Gerade parallel zu Deckel aussen vorbei, Abstand Zylinderachse > r';
      case 7
        ug = [1 0 0]';
        rg = [0 2 2]';
        pt_groundtruth_0 = [0 1 2]';
        intersect_truth = false;
        tt = 'Gerade parallel zu Deckel mittig vorbei';
      case 8
        ug = [1 0 0]';
        rg = [0 1 2]';
        pt_groundtruth_0 = [0 1 2]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Deckel tangential am Mantel';
      case 9
        ug = [1 0 1]';
        rg = [0 0 2]';
        pt_groundtruth_0 = [-1 0 1]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt 2x Mantel';
      case 10
        ug = [1 0 1]';
        rg = [0 0 5]';
        pt_groundtruth_0 = [-1 0 4]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt Mantel und Deckel';
      case 11
        ug = [1 0 1/5]';
        rg = [0 0 5/2]';
        pt_groundtruth_0 = [-1/2 0 0]';
        intersect_truth = true;
        tt = 'Gerade windschief Schnitt beide Deckel';
      case 12
        ug = [1 0 1]';
        rg = [0 1 2]';
        pt_groundtruth_0 = [0 1 2]';
        intersect_truth = false;
        tt = 'Gerade windschief tangential';
      case 13
        ug = [1 0 0]';
        rg = [0 2 2]';
        pt_groundtruth_0 = [0 1 2]';
        intersect_truth = false;
        tt = 'Gerade windschief mittig vorbei';
      case 14
        ug = [1 0 0]';
        rg = [0 2 2]';
        pt_groundtruth_0 = [0 1 2]';
        intersect_truth = false;
        tt = 'Gerade windschief aussen vorbei über Deckel';
      case 15
        ug = [1 0 0]';
        rg = [0 2 2]';
        pt_groundtruth_0 = [0 1 2]';
        intersect_truth = false;
        tt = 'Gerade windschief aussen vorbei nicht über Deckel';
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
      cyl_W = [eye(3,4)*T_W_0*[cyl_0(1:3)';1]; eye(3,4)*T_W_0*[cyl_0(4:6)';1]; cyl_0(7)]';
      pkol_groundtruth_W = (eye(3,4)*T_W_0*[pt_groundtruth_0(:);1])';
      rg_W = eye(3,4)*T_W_0*[rg;1];
      ug_W = T_W_0(1:3,1:3)*ug;
      %% Schnitt berechnen
      S_tmp = find_intersection_line_cylinder(rg_W, ug_W, cyl_W(1:3)', cyl_W(4:6)', cyl_W(7));
      if any(isnan(S_tmp(:,2)))
        pt1 = S_tmp(:,1);
        pt2 = pt1;
        dist = S_tmp(1,2);
      else
        ind = find(max(abs(ug_W))==abs(ug_W));
        lambda1 = S_tmp(ind,1)/ug_W(ind);
        lambda2 = S_tmp(ind,2)/ug_W(ind);
        if lambda1 < lambda2
          pt1 = S_tmp(:,1);
          pt2 = S_tmp(:,2);
        else
          pt1 = S_tmp(:,2);
          pt2 = S_tmp(:,1);
        end
        dist = 0;
      end
      %% Zeichnen
      change_current_figure(i);clf; hold on;
      drawCylinder(cyl_W, 'EdgeColor', 'k', 'FaceColor', 'b', 'FaceAlpha', 0.3);
      plot3(pt1(1), pt1(2), pt1(3), 'kx', 'MarkerSize', 10, 'LineWidth', 3);
      plot3(pt2(1), pt2(2), pt2(3), 'r+', 'MarkerSize', 12, 'LineWidth', 3);
      plot3([pkol_groundtruth_W(1);pt1(1)], [pkol_groundtruth_W(2);pt1(2)], ...
            [pkol_groundtruth_W(3);pt1(3)], '--c', 'MarkerSize', 5, 'LineWidth', 3);
      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
      view(3); grid on; axis equal;
      title(sprintf('Fall %d: %s. Abstand: %1.2f', i, tt, dist));
      mm = minmax2([pt1';pt2';pkol_groundtruth_W; cyl_W(1:3); cyl_W(4:6)]');
      lambda_min = (-2 + mm(:,1) - rg_W)./ug_W;
      lambda_max = (2 + mm(:,2) - rg_W)./ug_W;
      lambda_min = max(lambda_min);
      lambda_max = min(lambda_max);
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
      if all(~isnan(pt_groundtruth_0))
        % Falls händisch ein Punkt bestimmt wurde
        assert(all(abs(pt1 - pkol_groundtruth_W') < testtol), ...
          'Händisch bestimmter nächster Punkt stimmt nicht mit Berechnung: [%s] vs [%s]', ...
          disp_array(pkol_groundtruth_W, '%1.3f'), disp_array(pt1', '%1.3f'));
      end
      % Prüfung gegen Ausgabe der mex-Funktion
      S_tmp2 = find_intersection_line_cylinder_mex(rg_W, ug_W, cyl_W(1:3)', ...
        cyl_W(4:6)', cyl_W(7));
      assert(all(abs(S_tmp(~isnan(S_tmp(:))) - S_tmp2(~isnan(S_tmp(:)))) < 1e-12), ...
        'Ausgabevariable S_tmp2 stimmt nicht mit mex-Funktion überein');
    end
  end
end
fprintf('Testfälle für Kollision Zylinder und Punkt erfolgreich absolviert.\n');