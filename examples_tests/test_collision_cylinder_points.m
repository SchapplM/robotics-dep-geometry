% Teste Funktionen für Kollisionsprüfung von Zylinder und Punkt
% 
% Ergebnis:
% Für alle erdachten Testfälle sieht das Ergebnis plausibel aus.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({ ...
  'distance_plane_point2', ...
  'collision_cylinder_point', ...
  });
%% Teste Kollision aus Quader und Punkt
for j = 1:2 % Schleife über Vertauschung der Enden
  % Vertausche die Reihenfolge der Punkte, um mehr Code in Test abzudecken.
  % Darf keinen Einfluss auf Ergebnis haben
  if j == 1
    cyl_0 = [0,0,0, 0,0,.5, .1];
  else
    cyl_0 = [0,0,.5, 0,0,0, .1];
  end
  for i = 1:10 % Schleife über manuelle Testszenarien
    switch i
      case 1
        pt_0 = [0.0,0.0,0.6];
        kol_groundtruth = false;
        pkol_groundtruth_0 = [0.0, 0.0 0.5];
        tt = 'Punkt außerhalb auf Mittellinie';
      case 2
        pt_0 = [0.0,0.0,0.5];
        kol_groundtruth = true;
        pkol_groundtruth_0 = pt_0;
        tt = 'Punkt auf Deckel auf Mittellinie';
      case 3
        pt_0 = [0.0, 0.0, 0.3];
        kol_groundtruth = false;
        pkol_groundtruth_0 = NaN(3,1);
        tt = 'Punkt innen auf Mittellinie, Mantel am nächsten';
      case 4
        pt_0 = [0.3, 0.4, 0.6];
        kol_groundtruth = false;
        pkol_groundtruth_0 = NaN(3,1);
        tt = 'Punkt außerhalb, Rand am nächsten';
      case 5
        pt_0 = [0.05, 0.03, 0.6];
        kol_groundtruth = false;
        pkol_groundtruth_0 = [pt_0(1), pt_0(2), 0.5];
        tt = 'Punkt außerhalb, Deckelfläche am nächsten';
      case 6
        pt_0 = [0.2, 0.3, 0.3];
        kol_groundtruth = false;
        phi = atan2(0.3, 0.2);
        pkol_groundtruth_0 = [.1*cos(phi), .1*sin(phi), 0.3];
        tt = 'Punkt außerhalb, Mantel am nächsten';
      case 7
        pt_0 = [0.05,-0.03,0.5];
        kol_groundtruth = true;
        pkol_groundtruth_0 = pt_0;
        tt = 'Punkt auf Deckel';
      case 8
        pt_0 = [-0.01, 0.05, 0.2];
        kol_groundtruth = true;
        pkol_groundtruth_0 = NaN(3,1);
        tt = 'Punkt innen außermittig, Mantel am nächsten';
      case 9
        pt_0 = [0.02, 0.01, 0.47];
        kol_groundtruth = true;
        pkol_groundtruth_0 = [0.02, 0.01, 0.5];
        tt = 'Punkt innen, Deckel am nächsten';
      case 10
        pt_0 = [0.1*sqrt(2)/2, 0.1*sqrt(2)/2, 0.5];
        kol_groundtruth = true;
        pkol_groundtruth_0 = pt_0;
        tt = 'Punkt auf Rand (Kante)';
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
      pt_W = T_W_0*[pt_0(:);1];
      pt = pt_W(1:3)';
      cyl_W = [eye(3,4)*T_W_0*[cyl_0(1:3)';1]; eye(3,4)*T_W_0*[cyl_0(4:6)';1]; cyl_0(7)]';
      pkol_groundtruth_W = (eye(3,4)*T_W_0*[pkol_groundtruth_0(:);1])';
      %% Kollision berechnen
      [dist, kol, pkol, dmin] = collision_cylinder_point(cyl_W, pt);

      %% Zeichnen
      change_current_figure(i);clf; hold on;
      drawCylinder(cyl_W, 'EdgeColor', 'k', 'FaceColor', 'b', 'FaceAlpha', 0.3);
      plot3(pt(1), pt(2), pt(3), '-kx', 'MarkerSize', 10, 'LineWidth', 3);
      plot3([pkol(1);pt(1)], [pkol(2);pt(2)], [pkol(3);pt(3)], '-g', 'MarkerSize', 5, 'LineWidth', 3);
      plot3([pkol_groundtruth_W(1);pt(1)], [pkol_groundtruth_W(2);pt(2)], ...
            [pkol_groundtruth_W(3);pt(3)], '--c', 'MarkerSize', 5, 'LineWidth', 3);
      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
      view(3); grid on; axis equal;
      title(sprintf('Fall %d: %s. Dist=%1.0fmm', i, tt, 1e3*dist));
      drawnow();
      %% Prüfung
      % Rechnerische Prüfung der Ergebnisse
      assert(all(size(pkol)==[1 3]), 'pkol muss 1x3 sein');
      assert(all(size(dmin)==[1 1]), 'dmin muss 1x1 sein');
      assert(all(~isnan([dist(:); pkol(:)])), 'Ausgabe sollte nicht NaN sein');
      assert(abs(norm(pkol(:)-pt(:)) - abs(dist)) < 1e-12, ...
        'Abstand und Kollisionspunkte stimmen nicht überein');
      % Prüfung aus vorheriger Visueller Überprüfung
      if dist > 1e-10 % Bei numerischen Fehlern Erkennung nicht immer möglich
        assert(kol == kol_groundtruth, ...
          'Erkannte Kollision stimmt nicht aus händischer Prüfung');
      end
      if all(~isnan(pkol_groundtruth_W))
        % Falls händisch ein Punkt bestimmt wurde
        assert(all(abs(pkol - pkol_groundtruth_W)<1e-10), ...
          'Händisch bestimmter nächster Punkt stimmt nicht mit Berechnung');
      end
      % Prüfung gegen Ausgabe der mex-Funktion
      [dist2, kol2, pkol2, dmin2] = collision_cylinder_point_mex(cyl_W, pt);
      assert(abs(dist - dist2) < 1e-12, ...
        'Ausgabevariable dist stimmt nicht mit mex-Funktion überein');
      if dist > 1e-10
        assert(abs(kol - kol2) < 1e-12, ...
          'Ausgabevariable kol stimmt nicht mit mex-Funktion überein');
      end
      assert(all(abs(pkol(:) - pkol2(:)) < 1e-12), ...
        'Ausgabevariable pkol stimmt nicht mit mex-Funktion überein');
      assert(abs(dmin - dmin2) < 1e-12, ...
        'Ausgabevariable dmin stimmt nicht mit mex-Funktion überein');
    end
  end
end
fprintf('Testfälle für Kollision Zylinder und Punkt erfolgreich absolviert.\n');