% Teste Funktionen für Kollisionsprüfung von Zylinder und Gerade
% 
% Ergebnis:
% * Fehler der Funktion find_intersection_line_cylinder in Grenzfall.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
% matlabfcn2mex({'find_intersection_line_cylinder'});
%% Teste Kollision aus Zylinder und Gerade
for j = 1:2 % Schleife über Vertauschung der Enden
  % Vertausche die Reihenfolge der Punkte, um mehr Code in Test abzudecken.
  % Darf keinen Einfluss auf Ergebnis haben
  if j == 1
    cyl_0 = [0,0,0, 0,0,.5, .1];
  elseif j == 2
    cyl_0 = [0,0,.5, 0,0,0, .1];
  end
  for i = 1:4 % Schleife über manuelle Testszenarien
    testtol = 1e-9;
    switch i
      case 1
        ug = [0 0 1]';
        rg = [0.0 0.0 0.6]';
        pt_groundtruth_0 = [0.0 0.0 0.5]';
        intersect_truth = true;
        tt = 'Gerade parallel zu Mittellinie';
      case 2
        ug = [0.2;0.3;.1];
        rg = [0.3;0.2;.3];
        pt_groundtruth_0 = [0.0832, -0.0555 0.2077]';
        testtol = 1e-3;
        intersect_truth = false;
        tt = 'Gerade schräg am Mantel vorbei';
      case 3
        ug = [10.4106 -0.8246 5.5835]';
        rg = [0.0790 0.9969 0.0000]';
        pt_groundtruth_0 = [0.0039, 0.0498 0]';
        testtol = 1e-3;
        intersect_truth = false;
        tt = 'Grenzfall Gerade parallel zu Deckel';
      case 4
        % Zusammen mit Fall 3: Durch numerische Fehler kommt in Fall 4 das
        % falsche Ergebnis raus. In Fall 3 ist es mit weniger Nachkomma-
        % stellen noch richtig.
        rg = [1.041056477517526879239540e+01 -8.246239553203905670031304e-01 5.583472920156343555220246e+00]';
        ug = [7.896296943597737316711260e-02 9.968775498815553026688008e-01 1.034123188456720543598260e-16]';
        pt_groundtruth_0 = [0.0039, 0.0498 0]';
        testtol = 1e-3;
        intersect_truth = false;
        tt = 'Grenzfall Gerade parallel zu Deckel';
      case  5
        ug = [ -6.12323399573676603587e-17 -1 -6.12323399573676603587e-17]';
        rg = [-1.02444169721682049046e-01 1+6.27289622704830598630e-18 3.0000]';
        pt_groundtruth_0 = [0.0039, 0.0498 0]';
        testtol = 1e-3;
        intersect_truth = false;
        tt = 'Grenzfall senkrechter Zylinder zu Gerade';
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
      %% Schnitt berechnen
      S_tmp = find_intersection_line_cylinder(rg, ug, cyl_W(1:3)', cyl_W(4:6)', cyl_W(7));
      pt1 = S_tmp(:,1);
      if any(isnan(S_tmp(:,2)))
        pt2 = pt1;
        dist = S_tmp(1,2);
      else
        pt2 = S_tmp(:,2);
        dist = 0;
      end
      %% Zeichnen
      change_current_figure(i);clf; hold on;
      drawCylinder(cyl_W, 'EdgeColor', 'k', 'FaceColor', 'b', 'FaceAlpha', 0.3);
      plot3([rg(1)-5*ug(1);rg(1)+5*ug(1)], ...
            [rg(2)-5*ug(2);rg(2)+5*ug(2)], ...
            [rg(3)-5*ug(3);rg(3)+5*ug(3)], 'b-');
      plot3(pt1(1), pt1(2), pt1(3), 'kx', 'MarkerSize', 10, 'LineWidth', 3);
      plot3(pt2(1), pt2(2), pt2(3), 'r+', 'MarkerSize', 12, 'LineWidth', 3);
      plot3([pkol_groundtruth_W(1);pt1(1)], [pkol_groundtruth_W(2);pt1(2)], ...
            [pkol_groundtruth_W(3);pt1(3)], '--c', 'MarkerSize', 5, 'LineWidth', 3);
      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
      view(3); grid on; axis equal;
      title(sprintf('Fall %d: %s. Abstand: %1.2f', i, tt, dist));
      mm = minmax2([pt1';pt2';pt_groundtruth_0'; cyl_W(1:3); cyl_W(4:6)]');
      xlim([-0.1+mm(1,1), 0.1+mm(1,2)]);
      ylim([-0.1+mm(2,1), 0.1+mm(2,2)]);
      zlim([-0.1+mm(3,1), 0.1+mm(3,2)]);
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
        assert(all(abs(pt1 - pt_groundtruth_0) < testtol), ...
          'Händisch bestimmter nächster Punkt stimmt nicht mit Berechnung: [%s] vs [%s]', ...
          disp_array(pt_groundtruth_0', '%1.3f'), disp_array(pt1', '%1.3f'));
      end
      % Prüfung gegen Ausgabe der mex-Funktion
      S_tmp2 = find_intersection_line_cylinder_mex(rg, ug, cyl_W(1:3)', ...
        cyl_W(4:6)', cyl_W(7));
      assert(all(abs(S_tmp(~isnan(S_tmp(:))) - S_tmp2(~isnan(S_tmp(:)))) < 1e-12), ...
        'Ausgabevariable S_tmp2 stimmt nicht mit mex-Funktion überein');
    end
  end
end
fprintf('Testfälle für Kollision Zylinder und Punkt erfolgreich absolviert.\n');