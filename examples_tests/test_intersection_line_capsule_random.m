% Teste Funktionen für Kollisionsprüfung von Kapsel und Gerade
% 
% Ergebnis:
% * Fehler in collision_capsule_capsule, wenn Kapsel-Mittellinie fast
%   parallel zu Geraden ist. Dann nächster Abstand stark falsch gefunden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({'find_intersection_line_capsule','collision_capsule_capsule'});
t1 = tic();
%% Abgleich von find_intersection_line_capsule und collision_capsule_capsule
% Wenn eine Kapsel als Linie degeneriert (Radius Null, Länge ausreichend),
% dann müssen beide Funktionen das gleiche Ergebnis haben
num_tests_max = 1e4;
num_tests = 0;
for k = 1:num_tests_max
  % Definition der Kapsel (zufällig)
  pt1 = [-1;-1;-1] + -0.5+rand(3,1); % Kapsel Punkt 1
  pt2 = [ 1; 1; 1] + -0.5+rand(3,1); % Kapsel Punkt 2
  r = 0.1; % Radius der Kapsel
  cap = [pt1', pt2', r];
  
  rg = -0.5+rand(3,1); % Aufpunkt der Geraden
  ug = -0.5+rand(3,1); % Richtungsvektor der Geraden
  % Prüfe Ort des kürzesten Abstands von Gerade und Gerade
  % (bzgl. Mittellinie der Kapsel)
  [~, ~, lambda, mu] = distance_line_line([pt1', (pt2-pt1)'/norm((pt2-pt1))], [rg', ug'/norm(ug)]);
  if abs(lambda) > 10 || abs(mu) > 10
    % Die Prüfung ist nicht möglich, da der kürzeste Abstand weit außerhalb
    % der Kapsel liegt. Die Approximation der Kapsel als Linie ist so nicht
    % möglich. Die Kapsel müsste dann länger sein. Ist aber nicht möglich.
    continue
  end
  num_tests = num_tests + 1;
  % Ersatz-Kapsel ohne Radius. Die Länge wird relativ groß gewählt, falls
  % bei fast parallelen Geraden der Schnittpunkt fast im unendlichen liegt.
  % Eine größere Wahl ist aus numerischen Gründen nicht möglich (Abweichung
  % bei mex-Aufruf)
  cap2 = [rg'-1e2*ug'/norm(ug), rg'+1e2*ug'/norm(ug), 0];
  % Berechnung
  pts = find_intersection_line_capsule(rg, ug, pt1, pt2, r);
  if isnan(pts(2,2)) % Nachverarbeitung der Ausgaben
    dist = pts(1,2);
    pkol = pts';
  else
    dist = 0;
    pkol = pts';
  end
  [dist2, kol2, pkol2, d_min2] = collision_capsule_capsule(cap, cap2);
  if dist2 < 0
    dist2 = 0; % Tiefe der Durchdringung bei anderer Fkt nicht verfügbar
  end
  % Testen
  raiseerr = false;
  if abs(dist-dist2) > 1e-9
    raiseerr = true;
    warning('Abstand bei beiden Funktionen unterschiedlich erkannt: %1.2f vs %1.2f', dist, dist2);
  end
  if any(abs(pkol(1,1:3) - pkol2(1,1:3)) > 1e-9)
    if ~kol2 % wenn es keine Durchdringung gibt, muss das Ergebnis identisch sein.
      raiseerr = true;
      warning('Erkannter Kollisionspunkt unterschiedlich');
    else
      % Dieser Fall muss nicht berücksichtigt werden. Beide Funktionen
      % haben unterschiedliche Ausgaben (Schnittpunkte bei Ein-/Austritt vs
      % Punkt der tiefsten Durchdringung).
    end
  end
  % Prüfe Mex-Funktionen
  pts_mex = find_intersection_line_capsule_mex(rg, ug, pt1, pt2, r);
  [dist2_mex, kol2_mex, pkol2_mex, d_min2_mex] = collision_capsule_capsule_mex(cap, cap2);
  if dist2_mex < 0
    dist2_mex = 0; % Tiefe der Durchdringung bei anderer Fkt nicht verfügbar
  end
  if any(abs(pts_mex(:)-pts(:)) > 1e-9)
    error('Ausgabe 1 von find_intersection_line_capsule stimmt nicht gegen mex');
  end
  if abs(dist2_mex-dist2) > 1e-9
    error('Ausgabe 1 von collision_capsule_capsule stimmt nicht gegen mex');
  end
  if kol2_mex~=kol2
    error('Ausgabe 2 von collision_capsule_capsule stimmt nicht gegen mex');
  end
  if any(abs(pkol2_mex(:)-pkol2(:)) > 1e-9)
    error('Ausgabe 3 von collision_capsule_capsule stimmt nicht gegen mex');
  end
  if abs(d_min2_mex-d_min2) > 1e-9
    error('Ausgabe 4 von collision_capsule_capsule stimmt nicht gegen mex');
  end
  % Prüfung, ob einer der erkannten Punkte auf der Kapsel liegt
  [dist_cp1, kol_cp1, pkol_cp1, d_min_cp1] = collision_capsule_sphere(cap, [pts(1:3,1)',0]);
  if ~(abs(dist_cp1) < 1e-10)
    error('Keiner der erkannten Punkte liegt auf der Kapsel');
  end
  if ~raiseerr
    continue;
  end
  figure(1); clf; hold on;
  drawCapsule(cap,'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', ':');
  drawCapsule(cap2,'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--');
  hdl1=plot3(pkol(:,1), pkol(:,2), pkol(:,3), '--kx', 'MarkerSize', 5, 'LineWidth', 3);
  hdl2=plot3(pkol2(:,1), pkol2(:,2), pkol2(:,3), '-go', 'MarkerSize', 5, 'LineWidth', 3);
  legend([hdl1;hdl2], {'find_intersection_line_capsule','collision_capsule_capsule'}, 'interpreter', 'none');
  mm = minmax2([pkol;pkol2]');
  xlim([-0.1+mm(1,1), 0.1+mm(1,2)]);
  ylim([-0.1+mm(2,1), 0.1+mm(2,2)]);
  zlim([-0.1+mm(3,1), 0.1+mm(3,2)]);
  if raiseerr
    error('Angehalten aufgrund von Fehler.');
  end
  continue;
  % Tue so, als ob die Kapsel eine Gerade wäre und berechne den Abstand der
  % beiden Geraden. Damit Aufdeckung von Fehlern bereits in diesem Teil.
  rh = pt1;
  uh = pt2-pt1;
  [dnorm, d, lambda, mu, pg, ph] = distance_line_line([rh', uh'], [rg', ug']);
  figure(2);clf;hold on %#ok<UNRCH>
  plot3([rg(1);rg(1)+ug(1)], [rg(2);rg(2)+ug(2)], [rg(3);rg(3)+ug(3)], 'b-');
  plot3([rh(1);rh(1)+uh(1)], [rh(2);rh(2)+uh(2)], [rh(3);rh(3)+uh(3)], 'r-');
  plot3([rg(1);rg(1)+lambda*ug(1)], [rg(2);rg(2)+lambda*ug(2)], [rg(3);rg(3)+lambda*ug(3)], 'b-');
  plot3([rh(1);rh(1)+mu*uh(1)], [rh(2);rh(2)+mu*uh(2)], [rh(3);rh(3)+mu*uh(3)], 'r-');
  % Ergebnis der Matlab-Funktion
  plot3(pg(1), pg(2), pg(3), 'gv', 'MarkerSize', 12);
  plot3(ph(1), ph(2), ph(3), 'gv', 'MarkerSize', 12);
  plot3([pg(1);ph(1)], [pg(2);ph(2)], [pg(3);ph(3)], 'g-');
end

fprintf(['Zwei verschiedene Funktionen für Schnitt Linie/Kapsel ', ...
  'miteinander verglichen (für %d/%d Konfigurationen). Dauer: %1.1fs\n'], ...
  num_tests, num_tests_max, toc(t1));
