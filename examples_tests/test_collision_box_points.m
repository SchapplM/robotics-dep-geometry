% Teste Funktionen für Kollisionsprüfung eines einzelnen Punktes
% 
% Ergebnis:
% Für alle erdachten Testfälle sieht das Ergebnis plausibel aus.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({ ...
  'distance_plane_point', ...
  'collision_box_point', ...
  });
%% Teste Kollision aus Quader und Punkt
box_0 = [0,0,0, 0.5,0,0, 0,0.3,0, 0,0,0.4];
for i = 1:6
  switch i
    case 1
      pt_0 = [0.7,0.2,0.1];
      dist_groundtruth = NaN;
      kol_groundtruth = false;
      pkol_groundtruth_0 = [0.5, 0.2, 0.1];
      tt = 'Punkt neben Seitenfläche';
    case 2
      pt_0 = [0.7,0.4,0.8];
      dist_groundtruth = NaN;
      kol_groundtruth = false;
      pkol_groundtruth_0 = [0.5, 0.3, 0.4];
      tt = 'Punkt schräg über Ecke';
    case 3
      pt_0 = [0.2, 0.4, 0.5];
      dist_groundtruth = NaN;
      kol_groundtruth = false;
      pkol_groundtruth_0 = [0.2, 0.3, 0.4];
      tt = 'Punkt über Kante';
    case 4
      pt_0 = [0.3, 0.2, 0.3];
      dist_groundtruth = -0.1;
      kol_groundtruth = true;
      % Die Lage des Punktes ist nicht eindeutig, da der Abstand zu zwei
      % Seiten gleich groß ist. Keine Prüfung möglich.
      pkol_groundtruth_0 = NaN(3,1); % [0.3, 0.3, 0.3];
      tt = 'Punkt im Quader';
    case 5
      pt_0 = [0.5, 0.3, 0.4];
      dist_groundtruth = NaN;
      kol_groundtruth = true;
      pkol_groundtruth_0 = pt_0;
      tt = 'Punkt auf Ecke';
    case 6
      pt_0 = [0.2, 0.3, 0.4];
      dist_groundtruth = NaN;
      kol_groundtruth = true;
      pkol_groundtruth_0 = pt_0;
      tt = 'Punkt auf Kante';
  end
  for k = 1:2 % zufällige Transformation aller Punkte
    if k == 1
      T_W_0 = eye(4);
    else
      phi = 180*2*(-0.5+rand(3,1)); % [-180,180]
      T_W_0 = eulerAnglesToRotation3d(phi(1),phi(2),phi(3))* ...
              createTranslation3d(rand(3,1));
    end
    pt_W = T_W_0*[pt_0(:);1];
    pt = pt_W(1:3)';
    b1_W = T_W_0*[box_0(1:3)';1];
    box = [b1_W(1:3); T_W_0(1:3,1:3)*box_0(4:6)'; ...
           T_W_0(1:3,1:3)*box_0(7:9)'; T_W_0(1:3,1:3)*box_0(10:12)']';
    pkol_groundtruth_W = T_W_0*[pkol_groundtruth_0(:);1];
    pkol_groundtruth = pkol_groundtruth_W(1:3)';
    %% Kollision berechnen
    [dist, kol, pkol] = collision_box_point(box, pt);

    %% Zeichnen
    q = box(1:3)'; u1 = box(4:6)'; u2 = box(7:9)'; u3 = box(10:12)';
    change_current_figure(i);clf; hold on;
    cubpar_c = q(:)+(u1(:)+u2(:)+u3(:))/2; % Mittelpunkt des Quaders
    cubpar_l = [norm(u1); norm(u2); norm(u3)]; % Dimension des Quaders
    cubpar_a = rotation3dToEulerAngles([u1(:)/norm(u1), u2(:)/norm(u2), u3(:)/norm(u3)])';
    drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', 'b', 'FaceAlpha', 0.3);
    plot3(pt(1), pt(2), pt(3), '-kx', 'MarkerSize', 10, 'LineWidth', 3);
    plot3([pkol(1);pt(1)], [pkol(2);pt(2)], [pkol(3);pt(3)], '-g', 'MarkerSize', 5, 'LineWidth', 3);
    plot3([pkol_groundtruth(1);pt(1)], [pkol_groundtruth(2);pt(2)], [pkol_groundtruth(3);pt(3)], '--c', 'MarkerSize', 5, 'LineWidth', 3);
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    view(3); grid on; axis equal;
    title(sprintf('Fall %d: %s. Dist=%1.0fmm', i, tt, 1e3*dist));
    drawnow();
    %% Prüfung
    % Rechnerische Prüfung der Ergebnisse
    assert(all(~isnan([dist(:); pkol(:)])), 'Ausgabe sollte nicht NaN sein');
    assert(abs(norm(pkol(:)-pt(:)) - abs(dist)) < 1e-12, ...
      'Abstand und Kollisionspunkte stimmen nicht überein');
    % Prüfung aus vorheriger Visueller Überprüfung
    if dist > 1e-10 % Bei numerischen Fehlern Erkennung nicht immer möglich
      assert(kol == kol_groundtruth, ...
        'Erkannte Kollision stimmt nicht aus händischer Prüfung');
    end
    if all(~isnan(pkol_groundtruth))
      assert(all(abs(pkol - pkol_groundtruth)<1e-10), ...
        'Händisch bestimmter nächster Punkt stimmt nicht mit Berechnung');
    end
    if ~isnan(dist_groundtruth)
      assert(abs(dist - dist_groundtruth)<1e-10, ...
        'Händisch bestimmter Abstand stimmt nicht mit Berechnung');
    end
    % Prüfung gegen Ausgabe der mex-Funktion
    [dist2, kol2, pkol2] = collision_box_point_mex(box, pt);
    assert(abs(dist - dist2) < 1e-12, ...
      'Ausgabevariable dist stimmt nicht mit mex-Funktion überein');
    if dist > 1e-10
      assert(abs(kol - kol2) < 1e-12, ...
        'Ausgabevariable kol stimmt nicht mit mex-Funktion überein');
    end
    assert(all(abs(pkol(:) - pkol2(:)) < 1e-12), ...
      'Ausgabevariable pkol stimmt nicht mit mex-Funktion überein');
  end
end
fprintf('Testfälle für Kollision Quader und Punkt erfolgreich absolviert.\n');
