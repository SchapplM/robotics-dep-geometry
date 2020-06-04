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
  if dnorm2 > 1e-12 % Referenzfunktion für parallele Geraden falsch
    assert(abs(dnorm-dnorm2) < 1e-12, 'Abstand stimmt nicht überein');
    assert(all(abs(pg-pg2') < 1e-12), 'Punkt auf Gerade 1 stimmt nicht überein');
    assert(all(abs(ph-ph2') < 1e-12), 'Punkt auf Gerade 1 stimmt nicht überein');
  end
  % Prüfung gegen Ausgabe der mex-Funktion. Bei schlechter Konditionierung
  % (starke Parallelität) kann es größere numerische Abweichungen geben.
  [dnorm2, d2, lambda2, mu2, pg2, ph2] = distance_line_line_mex([rg', ug'], [rh', uh']);
  assert(all(abs(dnorm - dnorm2) < 1e-10), ...
    sprintf(['Ausgabevariable dnorm stimmt nicht mit mex-Funktion überein. ', ...
    'Fehler %1.2f'], dnorm-dnorm2));
  assert(all(abs(d - d2) < 1e-10), ...
    sprintf(['Ausgabevariable d stimmt nicht mit mex-Funktion überein. ', ...
    'Max Fehler %1.2e'], max(abs(d-d2))));
  test_mulambda = [lambda;mu] - [lambda2;mu2];
  assert(all(abs(test_mulambda) < 1e-10), ...
    sprintf(['Ausgabevariable lambda/mu stimmt nicht mit mex-Funktion überein. ', ...
    'Fehler: %1.2e/%1.2e'], test_mulambda(1), test_mulambda(2)));
  test_pgh = [pg;ph] - [pg2;ph2];
  assert(all(abs(test_pgh) < 1e-10), ...
    sprintf(['Ausgabevariable pg/ph stimmt nicht mit mex-Funktion überein. ', ...
    'Max. Fehler %1.2e'], max(abs(test_pgh))));
  
  continue % Folgender Teil nur zum Debuggen
  figure(1);clf;hold on %#ok<UNRCH>
  plot3([rg(1);rg(1)+ug(1)], [rg(2);rg(2)+ug(2)], [rg(3);rg(3)+ug(3)], 'b-');
  plot3([rh(1);rh(1)+uh(1)], [rh(2);rh(2)+uh(2)], [rh(3);rh(3)+uh(3)], 'r-');
  plot3([rg(1);rg(1)+lambda*ug(1)], [rg(2);rg(2)+lambda*ug(2)], [rg(3);rg(3)+lambda*ug(3)], 'b-');
  plot3([rh(1);rh(1)+mu*uh(1)], [rh(2);rh(2)+mu*uh(2)], [rh(3);rh(3)+mu*uh(3)], 'r-');
  % Ergebnis der Matlab-Funktion
  plot3(pg(1), pg(2), pg(3), 'gv', 'MarkerSize', 12);
  plot3(ph(1), ph(2), ph(3), 'gv', 'MarkerSize', 12);
  plot3([pg(1);ph(1)], [pg(2);ph(2)], [pg(3);ph(3)], 'g-');
  % Ergebnis der Mex-Funktion
  plot3(pg2(1), pg2(2), pg2(3), 'cs', 'MarkerSize', 12);
  plot3(ph2(1), ph2(2), ph2(3), 'cs', 'MarkerSize', 12);
  plot3([pg2(1);ph2(1)], [pg2(2);ph2(2)], [pg2(3);ph2(3)], 'c-');
end

%% Teste Kollision Kapsel-Kapsel
% Auswahl von Endpunkten für beliebige Geraden
A = [0.5 0.8 0.9 ;...
    -0.1 0.5 -0.4 ;...
    0 0.2 0.4 ];
B = [0.8 0.9 0.6;...
     0.5 -0.4 0.5;...
     0.2 0.4 0.1];

for i = 1:18 % Schleife über manuelle Testszenarien
  for j = 1:4 % Schleife über Vertauschung der Enden
    p1A = [0.0;0.0;0.0];
    p1B = [0.0;0.0;0.4];
    r1  = 0.1;
    switch i
      case 1
        tt = 'Keine Kollision. Verbindung Zylinder-Halbkugel';
        p2A = [0.2;0.3;0.2];
        p2B = [0.5;0.4;0.3];
        r2  = 0.2;
        kol_groundtruth = false;
      case 2
        tt = 'Kollision Zylinder-Kugelende';
        p2A = [0.1;0.2;0.2];
        p2B = [0.4;0.3;0.3];
        r2  = 0.2;
        kol_groundtruth = true;
      case 3
        tt = 'Kollision der Zylinder';
        p2A = [ 0.2;0.1;0.2];
        p2B = [-0.1;0.2;0.3];
        r2  = 0.08;
        kol_groundtruth = true;
      case 4
        tt = 'Kollision mit Zylinder und Kugelende';
        p2A = [0.1;0.2;0.35];
        p2B = [0.4;0.3;0.45];
        r2  = 0.2;
        kol_groundtruth = true;
      case 5
        tt = 'Kollision mit Zylinder und Kugelende. Parallele Zylinder';
        p2A = [0.1;0.1;0.6];
        p2B = [0.1;0.1;1.2];
        r2  = 0.3;
        kol_groundtruth = true;
      case 6 % Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Kontakt an näherem Ende.
        tt = 'Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Kontakt an näherem Ende';
        p2A = [0.1;0.1;0.6];
        p2B = [0.105;0.105;1.2];
        r2  = 0.3;
        kol_groundtruth = true;
      case 7 % Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Kontakt in Zylinder
        tt = 'Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Kontakt in Zylinder';
        p2A = [0.1;0.1;0.4];
        p2B = [0.105;0.105;1.0];
        r2  = 0.3;
        kol_groundtruth = true;
      case 8 % Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Fast vollständige Durchdringung (bis zum anderen Ende)
        tt = 'Kollision mit Zylinder und Kugelende. Fast parallele Zylinder. Fast vollständige Durchdringung (bis zum anderen Ende)';
        p2A = [0.1;0.1;0.1];
        p2B = [0.105;0.105;0.7];
        r2  = 0.2;
        kol_groundtruth = true;
      case 9 % Kollision mit Zylinder und Kugelende. Schräge starke Durchdringung
        % Die Zylinder-Achsen schneiden sich
        tt = 'Kollision mit Zylinder und Kugelende. Schräge starke Durchdringung';
        p2A = [0.1;0.1;0.1];
        p2B = [-0.2; -0.2; 0.7];
        r2  = 0.2;
        kol_groundtruth = true;
      case 10 % Eine Kapsel vollständig in anderer enthalten
        tt = 'Eine Kapsel vollständig in anderer enthalten (leicht schräg)';
        p2A = [0.1;0.1;0.1];
        p2B = [-0.1; -0.1; 0.7];
        r2  = 0.7;
        kol_groundtruth = true;
      case 11
        tt = 'Windschiefe Geraden';
        p1A = A(:,1); p1B = B(:,1);
        p2A = A(:,3); p2B = B(:,3);
        r1  = 0.001; r2  = 0.001;
        kol_groundtruth = false;
      case 12
        tt = 'Geraden mit einem gemeinsamen Punkt';
        p1A = A(:,1); p1B = B(:,1);
        p2A = A(:,2); p2B = B(:,2);
        r1  = 0.001; r2  = 0.001;
        kol_groundtruth = true;
      case 13
        tt = 'Geraden mit einem anderen gemeinsamen Punkt';
        p1A = A(:,2); p1B = B(:,2);
        p2A = A(:,3); p2B = B(:,3);
        r1  = 0.001; r2  = 0.001;
        kol_groundtruth = true;
      case 14
        tt = 'Komplett identische Geraden';
        p1A = A(:,2); p1B = B(:,2);
        p2A = A(:,3); p2B = B(:,3);
        r1  = 0.001; r2  = 0.001;
        kol_groundtruth = true;
      case 15
        tt = 'Keine Kollision. Verbindung Zylinder-Zylinder';
        p2A = [0.5;0.4;0.2];
        p2B = [0.1;0.6;0.3];
        r2 = 0.15;
        kol_groundtruth = false;
      case 16
        tt = 'Keine Kollision. Verbindung Halbkugel-Halbkugel';
        p2A = [0.1;0.1;0.7];
        p2B = [0.5;0.5;1.3];
        r2 = 0.2;
        kol_groundtruth = false;
      case 17
        tt = 'Komplett identisch und selber Ort';
        p2A = p1A;
        p2B = p1B;
        r2 = r1;
        kol_groundtruth = true;
      case 18
        tt = 'Eine Kapsel ist zur Kugel degeneriert';
        p2A = [0.1;0.1;0.7];
        p2B = p2A;
        r2 = 0.1;
        kol_groundtruth = false;
    end
    % Definieren der Eingabevariable für die Funktion. Zusätzlich
    % Vertausche die Reihenfolge der Punkte, um mehr Code in Test
    % abzudecken. Darf keinen Einfluss auf Ergebnis haben.
    if j <= 2, Kap1_0 = [p1A; p1B; r1]';
    else,      Kap1_0 = [p1B; p1A; r1]'; end
    % Alle 4 Kombinationen für die beiden Endpunkte beider Kapseln
    if any(j == [1 3]), Kap2_0 = [p2A; p2B; r2]';
    else,               Kap2_0 = [p2B; p2A; r2]'; end
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
      Kap2_W = [eye(3,4)*T_W_0*[Kap2_0(1:3)';1]; eye(3,4)*T_W_0*[Kap2_0(4:6)';1]; Kap2_0(7)]';
      p1A_W = eye(3,4)*T_W_0*[p1A;1]; p1B_W = eye(3,4)*T_W_0*[p1B;1];
      p2A_W = eye(3,4)*T_W_0*[p2A;1]; p2B_W = eye(3,4)*T_W_0*[p2B;1];
      %% Kollision berechnen
      [dist, kol, pkol, d_min] = collision_capsule_capsule(Kap1_W, Kap2_W);
      % Zusätzlich mex-Funktion berechnen (anderes Ergebnis möglich)
      [dist2, kol2, pkol2, d_min2] = collision_capsule_capsule_mex(Kap1_W, Kap2_W);
      %% Zeichnen
      if j == 1 % Nur für ersten Fall zeichnen.
        change_current_figure(i); clf; hold on;
        set(i, 'Name', sprintf('%02d', i), 'NumberTitle', 'off');
        plot3(p1A_W(1), p1A_W(2), p1A_W(3), 'kv', 'MarkerSize', 10);
        plot3(p1B_W(1), p1B_W(2), p1B_W(3), 'k^', 'MarkerSize', 10);
        drawCapsule(Kap1_W,'FaceColor', 'b', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', ':');

        plot3(p2A_W(1), p2A_W(2), p2A_W(3), 'ks', 'MarkerSize', 10);
        plot3(p2B_W(1), p2B_W(2), p2B_W(3), 'ko', 'MarkerSize', 10);
        drawCapsule(Kap2_W,'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'k', 'LineStyle', '--');

        plot3(pkol(:,1), pkol(:,2), pkol(:,3), '-kx', 'MarkerSize', 5, 'LineWidth', 3);
        plot3(pkol2(:,1), pkol2(:,2), pkol2(:,3), '-go', 'MarkerSize', 5, 'LineWidth', 3);
        xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
        view(3); grid on; axis equal;
        title(sprintf('Fall %d: %s', i, tt));
        drawnow();
      end

      %% Prüfung
      % Rechnerische Prüfung der Ergebnisse
      assert(all(~isnan([dist(:); pkol(:); d_min(:)])), 'Ausgabe sollte nicht NaN sein');
      assert(abs(norm(pkol(1, :) - pkol(2, :)) - abs(dist)) < 1e-12, ...
        'Abstand und Kollisionspunkte stimmen nicht überein');
      assert(all(size(pkol)==[2 3]), 'Ausgabe pkol muss 2x3 sein');
      assert(abs(norm(pkol2(1, :) - pkol2(2, :)) - abs(dist2)) < 1e-12, ...
        'Abstand und Kollisionspunkte stimmen nicht überein (mex-Funktion');
      assert((dist<0) == kol, 'Negativer Abstand ohne gemeldete Kollision (oder umgekehrt)');
      % Prüfung aus vorheriger Visueller Überprüfung
      assert(kol == kol_groundtruth, 'Erkannte Kollision stimmt nicht aus händischer Prüfung');
      % Prüfung nochmal die Abstandsberechnung der Zylinder-Mittellinie.
      % Damit sind numerische Abweichungen nachvollziehbar, falls oben ein
      % Fehler entsteht.
      rg = Kap1_W(1:3)'; rh = Kap2_W(1:3)'; % Anfangspunkte der Geraden
      ug = Kap1_W(4:6)'-Kap1_W(1:3)'; uh = Kap2_W(4:6)'-Kap2_W(1:3)'; % Richtungsvektoren der Geraden
      [dnorm, d, lambda, mu, pg, ph] = distance_line_line([rg', ug'], [rh', uh']);
      [dnorm2, d2, lambda2, mu2, pg2, ph2] = distance_line_line_mex([rg', ug'], [rh', uh']);

      % Prüfung gegen Ausgabe der mex-Funktion
      % Bei numerischen Fehlern (teilen durch eps) können unterschiedliche
      % Werte entstehen. Ebenso bei Fallunterscheidungen, wenn in
      % mex-Funktion durch andere Berechnung um eps unterschiedliche Werte
      % entstehen.
      assert(abs(norm(dist(:) - dist2(:))) < 1e-12, ...
        'Ausgabevariable dist stimmt nicht mit mex-Funktion überein');
      assert(abs(norm(kol(:) - kol2(:))) < 1e-12, ...
        'Ausgabevariable kol stimmt nicht mit mex-Funktion überein');
      if dnorm > 1e-12
        % Für den Fall, dass beide Mittellinien sich schneiden, ist das
        % Problem schlecht definiert und aufgrund von Rundungsfehlern treten
        % unterschiedliche Fallunterscheidungen in mex und script auf.
        % TODO: Code robuster machen.
        assert(abs(norm(pkol(:) - pkol2(:))) < 1e-12, ...
          'Ausgabevariable pkol stimmt nicht mit mex-Funktion überein');
      end
      assert(abs(norm(d_min(:) - d_min2(:))) < 1e-12, ...
        'Ausgabevariable d_min stimmt nicht mit mex-Funktion überein');
    end
  end
end
fprintf('Testszenarien für Kollision Kapsel-Kapsel erfolgreich absolviert\n');