% Teste Kollision zweier Kapseln sowie dafür notwendige Hilfsfunktionen
% 
% Ergebnis:
% Für alle erdachten Testfälle sieht das Ergebnis halbwegs plausibel aus.
% Gezeichnet werden die jeweiligen Kollisionsobjekte und ihre größte
% Durchdringung
% Die Kollisionserkennung scheint immer zu funktionieren
% Für einige Testfälle sieht es aber so aus, als ob die berechneten
% Kollisionspunkte nicht die Linie der größten Durchdringung darstellen.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({ ...
  'distance_line_line', ...
  'collision_capsule_capsule', ...
  });
%% Teste Hilfsfunktion für Abstand zweier Geraden
% Zufällige Daten
rg_ges = -0.5+rand(100,3);
ug_ges = -0.5+rand(100,3);
rh_ges = -0.5+rand(100,3);
uh_ges = -0.5+rand(100,3);
% Sonderfälle einbauen
ug_ges(92,:) = 0; % Richtungsvektor Null
uh_ges(93,:) = 0;
uh_ges(94,:) = 0; ug_ges(94,:) = 0;
rg_ges([95,97,99],:) = rh_ges([95,97,99],:); % Gleicher Startpunkt
ug_ges(96:97,:) = uh_ges(96:97,:); % Parallele Geraden
ug_ges(98:99,:) = -uh_ges(98:99,:); % Parallele Geraden (gegensätzlich def.)
for i = 1:100
  rg = rg_ges(i,:)';
  ug = ug_ges(i,:)';
  rh = rh_ges(i,:)';
  uh = uh_ges(i,:)';
  [dnorm, d, lambda, mu, pg, ph] = distance_line_line([rg', ug'], [rh', uh']);
  [dnorm2, pg2, ph2] = distanceLines3d([rg', ug'], [rh', uh']);
  if all(ug == 0) ||  all(uh == 0)
    continue % Fall nicht definiert. Immerhin kein Absturz
  end
  if dnorm2 ~= 0 % Referenzfunktion für parallele Geraden falsch
    assert(abs(dnorm-dnorm2) < 1e-12, 'Abstand stimmt nicht überein');
    assert(all(abs(pg-pg2') < 1e-12), 'Punkt auf Gerade 1 stimmt nicht überein');
    assert(all(abs(ph-ph2') < 1e-12), 'Punkt auf Gerade 1 stimmt nicht überein');
  end
  % Prüfung gegen Ausgabe der mex-Funktion
  [dnorm2, d2, lambda2, mu2, pg2, ph2] = distance_line_line_mex([rg', ug'], [rh', uh']);
  assert(all(abs(dnorm - dnorm2) < 1e-12), ...
    'Ausgabevariable dnorm stimmt nicht mit mex-Funktion überein');
  assert(all(abs(d - d2) < 1e-12), ...
    'Ausgabevariable d stimmt nicht mit mex-Funktion überein');
  assert(all(abs([lambda;mu] - [lambda2;mu2]) < 1e-12), ...
    'Ausgabevariable mu/lambda stimmt nicht mit mex-Funktion überein');
  assert(all(abs([pg;ph] - [pg2;ph2]) < 1e-12), ...
    'Ausgabevariable pg/ph stimmt nicht mit mex-Funktion überein');
  
  continue % Folgender Teil nur zum Debuggen
  figure(1);clf;hold on %#ok<UNRCH>
  plot3([rg(1);rg(1)+ug(1)], [rg(2);rg(2)+ug(2)], [rg(3);rg(3)+ug(3)], 'b-');
  plot3([rh(1);rh(1)+uh(1)], [rh(2);rh(2)+uh(2)], [rh(3);rh(3)+uh(3)], 'r-');
  plot3(pg(1), pg(2), pg(3), 'gv', 'MarkerSize', 12);
  plot3(ph(1), ph(2), ph(3), 'gv', 'MarkerSize', 12);
  plot3([pg(1);ph(1)], [pg(2);ph(2)], [pg(3);ph(3)], 'k-');
end

%% Teste Kollision Kapsel-Kugel
for i = 1:10
  for j = 1:4

    p1A = [0.0;0.0;0.0];
    p1B = [0.0;0.0;0.4];
    r1  = 0.1;
    % Vertausche die Reihenfolge der Punkte, um mehr Code in Test abzudecken.
    % Darf keinen Einfluss auf Ergebnis haben
    if j <= 2
      Kap1 = [p1A; p1B; r1]';
    else
      Kap1 = [p1B; p1A; r1]';
    end
    switch i
      case 1 % Keine Kollision
        p2A = [0.2;0.3;0.2];
        p2B = [0.5;0.4;0.3];
        r2  = 0.2;
        kol_groundtruth = false;
      case 2 % Kollision Zylinder-Kugelende
        p2A = [0.1;0.2;0.2];
        p2B = [0.4;0.3;0.3];
        r2  = 0.2;
        kol_groundtruth = true;
      case 3 % Kollision der Zylinder
        p2A = [ 0.2;0.1;0.2];
        p2B = [-0.1;0.2;0.3];
        r2  = 0.08;
        kol_groundtruth = true;
      case 4 % Kollision mit Zylinder und Kugelende
        p2A = [0.1;0.2;0.35];
        p2B = [0.4;0.3;0.45];
        r2  = 0.2;
        kol_groundtruth = true;
      case 5 % Kollision mit Zylinder und Kugelende. Parallele Zylinder.
        p2A = [0.1;0.1;0.6];
        p2B = [0.1;0.1;1.2];
        r2  = 0.3;
        kol_groundtruth = true;
      case 6 % Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Kontakt an näherem Ende.
        p2A = [0.1;0.1;0.6];
        p2B = [0.105;0.105;1.2];
        r2  = 0.3;
        kol_groundtruth = true;
      case 7 % Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Kontakt in Zylinder
        p2A = [0.1;0.1;0.4];
        p2B = [0.105;0.105;1.0];
        r2  = 0.3;
        kol_groundtruth = true;
      case 8 % Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Fast vollständige Durchdringung (bis zum anderen Ende)
        p2A = [0.1;0.1;0.1];
        p2B = [0.105;0.105;0.7];
        r2  = 0.2;
        kol_groundtruth = true;
      case 9 % Kollision mit Zylinder und Kugelende. Schräge starke Durchdringung
        % Die Zylinder-Achsen schneiden sich
        p2A = [0.1;0.1;0.1];
        p2B = [-0.2; -0.2; 0.7];
        r2  = 0.2;
        kol_groundtruth = true;
      case 10 % Eine Kapsel vollständig in anderer enthalten
        p2A = [0.1;0.1;0.1];
        p2B = [-0.1; -0.1; 0.7];
        r2  = 0.7;
        kol_groundtruth = true;
    end
    % Nochmal vertauschen der Punkte zur größeren Testabdeckung
    if any(j == [1 3])
      Kap2 = [p2A; p2B; r2]';
    else
      Kap2 = [p2B; p2A; r2]';
    end

    %% Kollision berechnen
    [dist, kol, pkol, d_min] = collision_capsule_capsule(Kap1, Kap2);
    % Zusätzlich mex-Funktion berechnen (anderes Ergebnis möglich)
    [dist2, kol2, pkol2, d_min2] = collision_capsule_capsule_mex(Kap1, Kap2);
    %% Zeichnen
    if j == 1 % Nur für ersten Fall zeichnen.
      figure(1); clf; hold on;
      plot3(p1A(1), p1A(2), p1A(3), 'kv', 'MarkerSize', 10);
      plot3(p1B(1), p1B(2), p1B(3), 'k^', 'MarkerSize', 10);
      drawSphere([p1A; r1]','FaceColor', 'b', 'FaceAlpha', 0.3);
      drawCylinder([p1A;p1B;r1]','FaceColor', 'b', 'FaceAlpha', 0.3);
      drawSphere([p1B; r1]','FaceColor', 'b', 'FaceAlpha', 0.3);

      plot3(p2A(1), p2A(2), p2A(3), 'ks', 'MarkerSize', 10);
      plot3(p2B(1), p2B(2), p2B(3), 'ko', 'MarkerSize', 10);
      drawSphere([p2A; r2]','FaceColor', 'r', 'FaceAlpha', 0.3);
      drawCylinder([p2A;p2B;r2]','FaceColor', 'r', 'FaceAlpha', 0.3);
      drawSphere([p2B; r2]','FaceColor', 'r', 'FaceAlpha', 0.3);

      plot3(pkol(:,1), pkol(:,2), pkol(:,3), '-kx', 'MarkerSize', 5, 'LineWidth', 3);
      plot3(pkol2(:,1), pkol2(:,2), pkol2(:,3), '-go', 'MarkerSize', 5, 'LineWidth', 3);
      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
      view(3); grid on;
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
    % Bei numerischen Fehlern (teilen durch eps) können unterschiedliche
    % Werte entstehen.
    assert(abs(norm(dist(:) - dist2(:))) < 1e-12, ...
      'Ausgabevariable dist stimmt nicht mit mex-Funktion überein');
    assert(abs(norm(kol(:) - kol2(:))) < 1e-12, ...
      'Ausgabevariable kol stimmt nicht mit mex-Funktion überein');
    assert(abs(norm(pkol(:) - pkol2(:))) < 1e-12, ...
      'Ausgabevariable pkol stimmt nicht mit mex-Funktion überein');
    assert(abs(norm(d_min(:) - d_min2(:))) < 1e-12, ...
      'Ausgabevariable d_min stimmt nicht mit mex-Funktion überein');
    % Prüfung nochmal die Abstandsberechnung der Zylinder-Mittellinie.
    % Damit sind numerische Abweichungen nachvollziehbar, falls oben ein
    % Fehler entsteht.
    rg = Kap1(1:3)'; rh = Kap2(1:3)'; % Anfangspunkte der Geraden
    ug = Kap1(4:6)'-Kap1(1:3)'; uh = Kap2(4:6)'-Kap2(1:3)'; % Richtungsvektoren der Geraden
    [dnorm, d, lambda, mu, pg, ph] = distance_line_line([rg', ug'], [rh', uh']);
    [dnorm2, d2, lambda2, mu2, pg2, ph2] = distance_line_line_mex([rg', ug'], [rh', uh']);
  end
end
