% Calculate intersections of a line with a capsule or the nearest point, if
% no intersection exists, it is assumed in the calculation, that the
% direction vectors u1,u2,u3 are pairwise orthogonal
% 
% Input:
% p [3x1]
%   line base vector
% u [3x1]
%   line direction vector
% q [3x1]
%   corner vector of box
% u1-u3 [3x1]
%   vectors to the beneighbouring corners
% Output:
% pts [3x2]
%   intersection points, or nearest point and distance to nearest point
%   stacked with NaNs, when no intersections exist

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function pts = find_intersection_line_box(p, u, q, u1, u2, u3)
  % Idee: homogene transformation der geraden und der Box in ein
  % Koordinatensystem, wo die box ein Einheitswürfel mit einer ecke im
  % Ursprung und einer bei [1 1 1] ist. Die inverse Rotationsmatrix
  % wird aus u1 u2 und u3 gebildet: R_inv = [u1 u2 u3] => diag(1/ui^2)*R_inv^T
  % damit müssen nur die Schnittpunkte der transformierten geraden mit den
  % ebenen x/y/z=0/1 berechnet werden und die erhaltenen Parameter nach
  % Größe geordnet werden. Liegen Schnittpunkte vor, gehüren die beiden
  % mittleren Parameter in der Liste zu diesen. Anderenfalls muss der zur
  % gerade nächste Punkt auf dem Würfel berechnet werden. Nach der
  % Berechnung muss noch die Rücktransformation durchgeführt werden.
  
  assert(isa(p,'double') && isreal(p) && all(size(p) == [3 1]), ...
    'find_intersection_line_box: p has to be [3x1] double');  
  assert(isa(u,'double') && isreal(u) && all(size(u) == [3 1]), ...
    'find_intersection_line_box: u has to be [3x1] double');  
  assert(isa(q,'double') && isreal(q) && all(size(q) == [3 1]), ...
    'find_intersection_line_box: q has to be [3x1] double');  
  assert(isa(u1,'double') && isreal(u1) && all(size(u1) == [3 1]), ...
    'find_intersection_line_box: u1 has to be [3x1] double');  
  assert(isa(u2,'double') && isreal(u2) && all(size(u2) == [3 1]), ...
    'find_intersection_line_box: u2 has to be [3x1] double');  
  assert(isa(u3,'double') && isreal(u3) && all(size(u3) == [3 1]), ...
    'find_intersection_line_box: u3 has to be [3x1] double');  
  
  R_inv = [u1, u2, u3];
  R     = diag([1/(u1.'*u1), 1/(u2.'*u2), 1/(u3.'*u3)])*R_inv.';
  pt    = R*(p-q);
  ut    = R*u;
  t     = NaN(1,6);
  t(1)  = -pt(1)/ut(1); % Ebene x=0
  t(2)  = -pt(2)/ut(2); % Ebene y=0
  t(3)  = -pt(3)/ut(3); % Ebene z=0
  t(4)  = (1-pt(1))/ut(1); % Ebene x=1
  t(5)  = (1-pt(2))/ut(2); % Ebene y=1
  t(6)  = (1-pt(3))/ut(3); % Ebene z=1
  t2    = sort(t(~isnan(t) & ~isinf(t)));
  

  i_krit = length(t2)/2;
  if (any(pt+t2(i_krit)*ut>1+1e-10) || any(pt+t2(i_krit)*ut<-1e-10) || any(pt+t2(i_krit+1)*ut>1+1e-10) || any(pt+t2(i_krit+1)*ut<-1e-10)) % keine Schnittpunkte
    % Finde nächsten Punkt zu den Zwölf Kanten und den 8 Ecken und nehme
    % das Minimum (bei den Kanten muss die Korrektheit geprüft werden, also
    % ob der Punkt auf der Kante liegt). Für die Ecken berechnen wir den
    % Abstand zur geraden mit d=|(e-p) x u|/|u|, für die Kanten e+ui*t mit
    % d=|(e-p)*(u x ui)|/|u x ui|.
    d_min = norm(cross(p-q,u))/norm(u);
    p_c = q;
    [d_min, p_c] = check_corner(q+u1,p,u,p_c,d_min);
    [d_min, p_c] = check_corner(q+u2,p,u,p_c,d_min);
    [d_min, p_c] = check_corner(q+u3,p,u,p_c,d_min);
    [d_min, p_c] = check_corner(q+u1+u2,p,u,p_c,d_min);
    [d_min, p_c] = check_corner(q+u2+u3,p,u,p_c,d_min);
    [d_min, p_c] = check_corner(q+u3+u1,p,u,p_c,d_min);
    [d_min, p_c] = check_corner(q+u1+u2+u3,p,u,p_c,d_min);
    cu1u = cross(u1,u);
    cu2u = cross(u2,u);
    cu3u = cross(u3,u);
    cucu1u = cross(u,cu1u);
    cucu2u = cross(u,cu2u);
    cucu3u = cross(u,cu3u);
    [d_min, p_c] = check_edge(q, u1, cu1u, cucu1u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u2, u1, cu1u, cucu1u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u3, u1, cu1u, cucu1u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u2+u3, u1, cu1u, cucu1u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q, u2, cu2u, cucu2u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u1, u2, cu2u, cucu2u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u3, u2, cu2u, cucu2u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u1+u3, u2, cu2u, cucu2u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q, u3, cu3u, cucu3u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u1, u3, cu3u, cucu3u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u2, u3, cu3u, cucu3u,p,p_c,d_min);
    [d_min, p_c] = check_edge(q+u1+u2, u3, cu3u, cucu3u,p,p_c,d_min);
    pts = [p_c, [norm(cross(p_c-p,u))/norm(u); NaN(2,1)]];
    pts = correct_pts(pts);
  else
    pts = [q+R_inv*(pt+t2(i_krit)*ut), q+R_inv*(pt+t2(i_krit+1)*ut)];
  end
end
  

% Hilfsfunktion, um zu Überprüfen, ob eine Ecke den akteullen
% Minmalabstand unterbietet
function [d_min, p_c] = check_corner(c,p,u,p_c,d_min)
  d = norm(cross(p-c,u))/norm(u);
  if d<d_min
    d_min = d;
    p_c = c;
  end
end

% Hilfsfunktion, um zu Überprüfen, ob eine Kante den aktuellen
% Minmalabstand unterbietet
function [d_min, p_c] = check_edge(e, ui, cuiu, cucuiu,p,p_c,d_min)
  d = norm((p-e).'*cuiu)/norm(cuiu);
  if d>eps % Wenn die gerade in einer Ebene zur Kante verläuft, ist das Ergebnis mit den Ecken abgedeckt, da ja kein regulärer SP vorliegt
    if d<d_min
      t_k = (p-e).'*cucuiu/(cuiu.'*cuiu);
      if t_k>-eps && t_k<1+eps % wenn t zwischen 0 und 1, dann liegt der kritische Punkt auf der Kante
        d_min = d;
        p_c = e + t_k*ui;
      end
    end
  end
end

% Hilfsfunktion zur Korrektur von Zylinderschnittpunkten die nurt numerisch
% ausserhalb des Zylinders liegen (d<1e-10)
function pts_out = correct_pts(pts_in)
  if isnan(pts_in(3,2)) && pts_in(1,2)<1e-10
    pts_out = [pts_in(1:3,1) pts_in(1:3,1)];
  else
    pts_out = pts_in;
  end
end
