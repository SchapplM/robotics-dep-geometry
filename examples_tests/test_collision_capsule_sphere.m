% Teste Kollision einer Kapsel mit einer Kugel
% 
% Ergebnis:
% Für alle erdachten Testfälle sieht das Ergebnis plausibel aus.
% Gezeichnet werden die jeweiligen Kollisionsobjekte und ihre größte
% Durchdringung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({ ...
  'distance_point_point', ...
  'distance_line_point', ...
  'collision_capsule_sphere', ...
  });

%% Teste Kollision aus Kugel und Kapsel
for i = 1:14 % Schleife über manuelle Testszenarien
  for j = 1:2 % Schleife über Vertauschung der Enden
    %% Parameter einstellen
    p1A = [0.0;0.0;0.0];
    p1B = [0.0;0.0;0.4];
    r1  = 0.1;
    switch i
      case 1
        tt = 'Keine Kollision. Auf Höhe Mantelfläche';
        p2 = [0.2;0.3;0.2];
        r2  = 0.15;
        kol_groundtruth = false;
      case 2
        tt = 'Keine Kollision. Auf Höhe Halbkugel';
        p2 = [0.3;-0.1;0.5];
        r2  = 0.15;
        kol_groundtruth = false;
      case 3
        tt = 'Keine Kollision. Auf Achse';
        p2 = [0.0; 0.0; 0.9];
        r2  = 0.15;
        kol_groundtruth = false;
      case 4
        tt = 'Kollision mit Mantelfläche';
        p2 = [0.2;0.3;0.2];
        r2  = 0.3;
        kol_groundtruth = true;
      case 5
        tt = 'Kollision mit Mantelfläche, mit Durchdringung der Halbkugeln';
        p2 = [0.2;0.3;0.2];
        r2  = 0.4;
        kol_groundtruth = true;
      case 6
        tt = 'Kollision mit Mantelfläche, Durchdringung der oberen Halbkugel';
        p2 = [0.2;0.4;0.3];
        r2  = 0.4;
        kol_groundtruth = true;
      case 7
        tt = 'Kollision mit oberer Halbkugel';
        p2 = [0.1;0.1;0.65];
        r2  = 0.2;
        kol_groundtruth = true;
      case 8
        tt = 'Kleine Kugel in Zylinder, außermittig';
        p2 = [0.04;0.03;0.2];
        r2  = 0.03;
        kol_groundtruth = true;
      case 9
        tt = 'Kleine Kugel in Zylinder, genau mittig';
        p2 = [0.00;0.00;0.2];
        r2  = 0.03;
        kol_groundtruth = true;
      case 10
        tt = 'Kleine Kugel in Zylinder, auf Mittellinie';
        p2 = [0.00;0.00;0.3];
        r2  = 0.04;
        kol_groundtruth = true;
      case 11
        tt = 'Kugel exakt in Halbkugel';
        p2 = p1A;
        r2  = 0.04;
        kol_groundtruth = true;
      case 12
        tt = 'Kugel exakt in Halbkugel, Kapsel schräg';
        p2 = p1A;
        p1B = [0.2;0.1;0.4];
        r2  = 0.04;
        kol_groundtruth = true;
      case 13
        tt = 'Kugel exakt in Halbkugel, Kapsel schräg von anderer Seite kommend';
        p2 = p1A;
        p1B = [0.2;0.1;-0.4];
        r2  = 0.04;
        kol_groundtruth = true;
      case 14
        tt = 'Kapsel ist zur Kugel degeneriert';
        p2 = [0.2;0.1;-0.4];
        p1B = p1A;
        r2  = 0.04;
        kol_groundtruth = false;
    end
    % Vertausche die Reihenfolge der Punkte, um mehr Code in Test abzudecken.
    % Darf keinen Einfluss auf Ergebnis haben
    if j <= 2
      Kap1_0 = [p1A; p1B; r1]';
    else
      Kap1_0 = [p1B; p1A; r1]';
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
      Kap1_W = [eye(3,4)*T_W_0*[Kap1_0(1:3)';1]; eye(3,4)*T_W_0*[Kap1_0(4:6)';1]; Kap1_0(7)]';
      p1A_W = eye(3,4)*T_W_0*[p1A;1]; p1B_W = eye(3,4)*T_W_0*[p1B;1];
      p2_W = eye(3,4)*T_W_0*[p2;1];
      %% Kollision berechnen
      [dist, kol, pkol, d_min] = collision_capsule_sphere(Kap1_W, [p2_W',r2]);
      %% Zeichnen
      if j == 1 % Nur für ersten Fall zeichnen.
        change_current_figure(i); clf; hold on;
        plot3(p1A_W(1), p1A_W(2), p1A_W(3), 'kv', 'MarkerSize', 10);
        plot3(p1B_W(1), p1B_W(2), p1B_W(3), 'k^', 'MarkerSize', 10);
        drawCapsule(Kap1_W,'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--')
        plot3(p2_W(1), p2_W(2), p2_W(3), 'ks', 'MarkerSize', 10);
        drawSphere([p2_W; r2]','FaceColor', 'r', 'FaceAlpha', 0.3, 'linestyle', ':');
        plot3(pkol(:,1), pkol(:,2), pkol(:,3), '-gx', 'MarkerSize', 5, 'LineWidth', 3)
        xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
        view(3); grid on; axis equal;
        title(sprintf('Fall %d: %s. Dist=%1.0fmm', i, tt, 1e3*dist));
      end
      %% Prüfung
      % Rechnerische Prüfung der Ergebnisse
      assert(all(~isnan([dist(:); pkol(:); d_min(:)])), 'Ausgabe sollte nicht NaN sein');
      assert(abs(norm(pkol(1, :) - pkol(2, :)) - abs(dist)) < 1e-12, ...
        'Abstand und Kollisionspunkte stimmen nicht überein');
      assert((dist<0) == kol, 'Negativer Abstand ohne gemeldete Kollision (oder umgekehrt)');
      % Prüfung aus vorheriger Visueller Überprüfung
      assert(kol == kol_groundtruth, 'Erkannte Kollision stimmt nicht aus händischer Prüfung');

      % Prüfung gegen Ausgabe der mex-Funktion
      [dist2, kol2, pkol2, d_min2] = collision_capsule_sphere_mex(Kap1_W, [p2_W',r2]);
      assert(abs(dist - dist2) < 1e-12, ...
        'Ausgabevariable dist stimmt nicht mit mex-Funktion überein');
      assert(abs(kol - kol2) < 1e-12, ...
        'Ausgabevariable kol stimmt nicht mit mex-Funktion überein');
      assert(all(abs(pkol(:) - pkol2(:)) < 1e-12), ...
        'Ausgabevariable pkol stimmt nicht mit mex-Funktion überein');
      assert(abs(d_min - d_min2) < 1e-12, ...
        'Ausgabevariable d_min stimmt nicht mit mex-Funktion überein');
    end
  end
end
fprintf('Testszenarien für Kollision Kapsel-Kugel erfolgreich absolviert\n');