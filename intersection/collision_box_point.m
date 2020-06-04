% Berechne Kollision aus Quader und Punkt
% Der Quader muss nicht Achsparallel sein.
% 
% Eingabe:
% box [q, u1, u2, u3]; 1x12
%   q: Aufpunkt der ersten Ecke des Quaders
%   u1,...: Vektoren zu den benachbarten Punkten zu q (Annahme: Rechtwinklig)
% p [1x3]
%   Koordinaten des zu prüfenden Punktes
% 
% Ausgabe:
% dist [1x1]
%   Abstand des Punktes zum Quader am Punkt größter Annäherung
% kol [1x1 logical]
%   true falls Punkt enthalten (entspricht Kollision), sonst false
% pkol [1x3]
%   Punkt des kürzesten Abstandes auf dem Quader

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dist, kol, pkol] = collision_box_point(box, p)
%#codegen
assert(isa(box,'double') && isreal(box) && all(size(box) == [1 12]) && ... 
       isa(p,'double') && isreal(p) && all(size(p) == [1 3]));

% Initialisierung
q = box(1:3);
u1 = box(4:6);
u2 = box(7:9);
u3 = box(10:12);

% Alle 8 Ecken prüfen
dnorm_corners_all = NaN(8,1);
d_corners_all = NaN(8,3);
corners = [q; q+u1; q+u2; q+u1+u2; q+u3; q+u1+u3; q+u2+u3; q+u1+u2+u3];
for i = 1:8
  [dnorm_corners_all(i,:), d_corners_all(i,:)] = distance_point_point(corners(i,:), p);
end

% Alle 12 Kanten des Quaders (je vier in Richtung u1/u2/u3)
edges = [ ...
  q, u1; q+u2, u1; q+u3, u1; q+u2+u3, u1; ...
  q, u2; q+u1, u2; q+u3, u2; q+u1+u3, u2; ...
  q, u3; q+u1, u3; q+u2, u3; q+u1+u2, u3];
dnorm_edges_all = NaN(12,1);
d_edges_all = NaN(12,3);
lambda_edges_all = NaN(12,1);
pg_edges_all = NaN(12,3);
for i = 1:12
  [dnorm_edges_all(i,:), d_edges_all(i,:), lambda_edges_all(i,:), ...
    pg_edges_all(i,:)] = distance_line_point(edges(i,:), p);
end
% Entferne Kanten, bei denen eine Verlängerung außerhalb des Quaders die
% kürzeste Verbindung darstellt
dnorm_edges_all(lambda_edges_all<0|lambda_edges_all>1) = NaN;

% Alle sechs Seitenflächen des Quaders
% Zuordnung der Ecken und der Kanten zu den jeweiligen Flächen
% Wenn man auf die Seite guckt, werden die Ecken und Kanten mathemathisch
% positiv von links unten beginnend gezählt. Die Kanten verbinden die Ecken
I_corners = [ ...
  [4 3 7 8]; ...
  [3 1 5 7]; ...
  [1 2 6 5]; ...
  [2 4 8 6]; ...
  [1 3 4 2]; ...
  [6 8 7 5]];
I_edges = [ ...
  [ 2 11  4 12]; ...
  [ 5  9  7 11]; ...
  [ 1 10  3  9]; ...
  [ 6 12  8 10]; ...
  [ 5  2  6  1]; ...
  [ 8  4  7  3]];
% Prüfe, ob eine Seitenfläche die nächste Fläche ist
dnorm_faces_all = NaN(6,1);
d_faces_all = NaN(6,3);
pe_faces_all = NaN(12,3);
kol = true;
for i = 1:6
  if ~all(lambda_edges_all(I_edges(i,:)) >= 0 & lambda_edges_all(I_edges(i,:)) <= 1)
    % Wenn Seitenfläche näher als Kante sein soll, muss der nächste Punkt
    % auf der Kante innerhalb der Ecken liegen (also zwischen 0 und 1 in
    % der Geradengleichung)
    kol = false; % Wenn der Punkt im Quader liegen soll, muss er Mittig bezogen auf alle Kanten liegen
    continue
  end
  Ebene = [corners(I_corners(i,1),:), edges(I_edges(i,1),4:6), edges(I_edges(i,2),4:6)];
  [dnorm_faces_all(i,:), d_faces_all(i,:), pe_faces_all(i,:)] = distance_plane_point(Ebene, p);
end
% Prüfe den kürzesten Abstand
[dist, Imin] = min([dnorm_corners_all; dnorm_edges_all; dnorm_faces_all]);
pkol_all = [corners; pg_edges_all; pe_faces_all];
pkol = pkol_all(Imin,:);
if kol % Bei Kollision wird eine negative Länge ausgegeben
  dist = -dist;
end
return
%% Debug: Indizes prüfen
for i = 1:6 %#ok<UNRCH>
  corners_i = corners(I_corners(i,:),:);
  edges_i = edges(I_edges(i,:),:);
  % Mittelpunkt
  center_from_corner = mean(corners_i);
  center_from_edges = mean(edges_i(:,1:3)+0.5*edges_i(:,4:6));
  test_center = center_from_edges-center_from_corner;
  if any(abs(test_center) > 1e-10)
    warning('Mittelpunkt Fläche %d stimmt nicht', i);
  end
end

%% Debug: Zeichnen
figure(100);clf; view(3); hold on;
cubpar_c = q(:)+(u1(:)+u2(:)+u3(:))/2; % Mittelpunkt des Quaders
cubpar_l = [norm(u1); norm(u2); norm(u3)]; % Dimension des Quaders
cubpar_a = 180/pi*r2eulzyx([u1(:)/norm(u1), u2(:)/norm(u2), u3(:)/norm(u3)]); % Orientierung des Quaders
drawCuboid([cubpar_c', cubpar_l', cubpar_a'], 'FaceColor', 'b', 'FaceAlpha', 0.3);
plot3(p(1), p(2), p(3), 'rx', 'MarkerSize', 10);
I_edgeselect = find(lambda_edges_all>0&lambda_edges_all<1);
for i = I_edgeselect'
  plot3([edges(i,1);edges(i,1)+edges(i,4)], ...
        [edges(i,2);edges(i,2)+edges(i,5)], ...
        [edges(i,3);edges(i,3)+edges(i,6)], 'm-', 'LineWidth', 5);
  plot3([pg_edges_all(i,1);pg_edges_all(i,1)+d_edges_all(i,1)], ...
        [pg_edges_all(i,2);pg_edges_all(i,2)+d_edges_all(i,2)], ...
        [pg_edges_all(i,3);pg_edges_all(i,3)+d_edges_all(i,3)], 'c-', 'LineWidth', 2);
  text(edges(i,1)+0.5*edges(i,4), ...
       edges(i,2)+0.5*edges(i,5), ...
       edges(i,3)+0.5*edges(i,6), sprintf('K%d', i));
end
for i = 1:8
  plot3(corners(i,1), corners(i,2), corners(i,3), 'ko', 'MarkerSize', 3);
  plot3([corners(i,1);corners(i,1)+d_corners_all(i,1)], ...
        [corners(i,2);corners(i,2)+d_corners_all(i,2)], ...
        [corners(i,3);corners(i,3)+d_corners_all(i,3)], 'r--', 'LineWidth', 1);
  text(corners(i,1), corners(i,2), corners(i,3), sprintf('E%d', i));
end
for i = 1:6
  corners_i = corners(I_corners(i,:),:);
  center_from_corner = mean(corners_i);
  text(center_from_corner(1), center_from_corner(2), center_from_corner(3), sprintf('S%d', i));
  plot3([pe_faces_all(i,1);pe_faces_all(i,1)+d_faces_all(i,1)], ...
        [pe_faces_all(i,2);pe_faces_all(i,2)+d_faces_all(i,2)], ...
        [pe_faces_all(i,3);pe_faces_all(i,3)+d_faces_all(i,3)], 'g--', 'LineWidth', 2);
end
