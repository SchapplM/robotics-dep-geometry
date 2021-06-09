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
%   intersection points, or nearest point on box and distance to
%   nearest point as well as line parameter until which the same distance is
%   kept if line is parallel to side planes stacked with NaNs,
%   when no intersections exist. The returned values are ordered such that the
%   value with the smaller s in x = p+s*u is returned in the first column.

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function pts = find_intersection_line_box(p, u, q, u1, u2, u3)
  % Idee: homogene transformation der Geraden und der Box in ein
  % Koordinatensystem, wo die Box ein Einheitswürfel mit einer Ecke im
  % Ursprung und einer bei [1 1 1] ist. Die inverse Rotationsmatrix
  % wird aus u1 u2 und u3 gebildet: R_inv = [u1 u2 u3] => diag(1/ui^2)*R_inv^T
  % damit müssen nur die Schnittpunkte der transformierten geraden mit den
  % ebenen x/y/z=0/1 berechnet werden und die erhaltenen Parameter nach
  % Größe geordnet werden. Liegen Schnittpunkte vor, gehören die beiden
  % mittleren Parameter in der Liste zu diesen. Anderenfalls muss der zur
  % gerade nächste Punkt auf dem Würfel berechnet werden. Nach der
  % Berechnung muss noch die Rücktransformation durchgeführt werden. Um den
  % nächstgelegenen Punkt zu bestimmen, wird untersucht, in welchen der 27
  % Quadranten, in die die Ebenen der Seitenflächen des Quaders den Raum
  % aufteilen die Gerade verlaeuft. Interessant sind nur diejenigen Quadranten
  % ab dem Verlassen des ersten vollstaendig durchlaufennen Zwischenbereichs
  % zwischen zwei parallelen Seitenebenen bis zum Eintritt in den letzen
  % durchlaufenen Zwischenbereich. Denn innerhalb des Zwischenbereiches aendert
  % sich der Abstand zum Quader in der entsprechenden Richtung nicht. D.h. vor
  % dem vollstaendigen Durchlaufen des ersten Zwischenbereichs, wird in zwei
  % Richtungen die Entfernung kleiner, da wir auf den Quader zulaufen und in
  % einer Richtung wird die Entfernung kleiner oder bleibt gleich, sodass wir
  % uns auf den Quader zubewegen. Nachdem Eintritt in den letzuten
  % Zwischenbereich wird die Entfernung in den zwei anderen Richtungen groesser,
  % waehrend sie in der jeweiligen Richtung gleich bleibt oder zunimmt. D.h. die
  % dazwischen durchlaufenen Sektoren enthalten den zum Quader naechstgelegenen
  % Punkt. Drei verschiedene Sektoren sind ausserhalb des Quaders zu betrachten:
  % ausserhalb aller Zwischenbereiche, der naechste Punkt ist die entsprechende
  % Ecke, innerhalb eines Zwischenbereiches, der naechste Punkt liegt auf der
  % Kante der beiden Begrenzungsebenen, die nicht den Zwischenbereich
  % definieren, in zwei Zwischenbereichen, der Fall kan ignoriert werden, da die
  % Entfernng in einem solchen Bereich bis zum Rand streng monoton verlaeuft und
  % der kritische Punkt somit in einem angrenzenden Sektor liegt.
  
  %#codegen
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
  
  u1_parallel = norm(u1.'*u)/(norm(u1)*norm(u)) < 1e-10;
  u2_parallel = norm(u2.'*u)/(norm(u2)*norm(u)) < 1e-10;
  u3_parallel = norm(u3.'*u)/(norm(u3)*norm(u)) < 1e-10;
  R_inv = [u1, u2, u3];
  R     = diag([1/(u1.'*u1), 1/(u2.'*u2), 1/(u3.'*u3)])*R_inv.';
  pt    = R*(p-q);
  ut    = R*u;
  t     = NaN(1,6);
  if ~u1_parallel
    t(1)  = -pt(1)/ut(1); % Ebene x=0
    t(2)  = (1-pt(1))/ut(1); % Ebene x=1
  end
  if ~u2_parallel
    t(3)  = -pt(2)/ut(2); % Ebene y=0
    t(4)  = (1-pt(2))/ut(2); % Ebene y=1
  end
  if ~u3_parallel
    t(5)  = -pt(3)/ut(3); % Ebene z=0
    t(6)  = (1-pt(3))/ut(3); % Ebene z=1
  end
  [t2, ind_t2] = sort(t(~isnan(t) & ~isinf(t)));
  

  i_krit = length(t2)/2;
  p_krit1 = pt+t2(i_krit)*ut;
  if any(p_krit1>1+1e-10) || any(p_krit1<-1e-10) % keine Schnittpunkte
    % Finde nächsten Punkt zu den Zwölf Kanten und den 8 Ecken und nehme
    % das Minimum (bei den Kanten muss die Korrektheit geprüft werden, also
    % ob der Punkt auf der Kante liegt). Für die Ecken berechnen wir den
    % Abstand zur geraden mit d=|(e-p) x u|/|u|, für die Kanten e+ui*t mit
    % d=|(e-p)*(u x ui)|/|u x ui|.
    if i_krit==1 % g parallel to two sides
      p_c = p_krit1;
      d_square = 0;
      if u1_parallel
        d_square = d_square + (max(min(p_c(1),0),p_c(1)-1)*norm(u1))^2;
        p_c(1) = min(1,max(0,p_c(1)));
      end
      if u2_parallel
        d_square = d_square + (max(min(p_c(2),0),p_c(2)-1)*norm(u2))^2;
        p_c(2) = min(1,max(0,p_c(2)));
      end
      if u3_parallel
        d_square = d_square + (max(min(p_c(3),0),p_c(3)-1)*norm(u3))^2;
        p_c(3) = min(1,max(0,p_c(3)));
      end
      p_krit2 = pt+t2(i_krit+1)*ut;
      pts = [q+R_inv*p_c, [sqrt(d_square); norm(R_inv*p_krit1 - R_inv*p_krit2); NaN]];
      pts = correct_pts(pts, u);
    elseif u1_parallel % g parallel to u1=const side only
      p_c = p_krit1([2 3]);
      if any(p_c>1+1e-10) || any(p_c<-1e-10) % nicht über Seite
        if p_krit1(1)>=1
          p_c = q+u1;
        elseif p_krit1(1)<=0
          p_c = q;
        else
          p_c = p_krit1;
          switch ind_t2(3)
            case 1 % y=0
              p_c(2) = 0;
            case 2 % y=1
              p_c(2) = 1;
            case 3 % z=0
              p_c(3) = 0;
            case 4 % z=1
              p_c(3) = 1;
          end
          p_c = q+R_inv*p_c;
          d = norm(cross(p_c-p,u))/norm(u);
          pts = [p_c, [d; NaN; NaN]];
          pts = correct_pts(pts, u);
          return;
        end
        switch ind_t2(2)*ind_t2(3)
          case 3 % y=0,z=0
          case 4 % y=0,z=1
            p_c = p_c + u3;
          case 6 % y=1,z=0
            p_c = p_c + u2;
          case 8 % y=1,z=1
            p_c = p_c + u2 + u3;
          otherwise
            error(['Error, line parallel to one side intersects opposite '...
                   'sides after another.']);
        end
        d = norm(cross(p_c-p,u))/norm(u);
        pts = [p_c, [d; NaN; NaN]];
        pts = correct_pts(pts, u);
      else
        p_c = p_krit1;
        p_krit2 = pt+t2(i_krit+1)*ut;
        d = abs(max(min(p_c(1),0),p_c(1)-1))*norm(u1);
        p_c(1) = min(1,max(0,p_c(1)));
        pts = [q+R_inv*p_c, [d; norm(R_inv*p_krit1 - R_inv*p_krit2); NaN]];
        pts = correct_pts(pts, u);
      end
    elseif u2_parallel % g parallel to u2=const side only
      p_c = p_krit1([1 3]);
      if any(p_c>1+1e-10) || any(p_c<-1e-10) % nicht über Seite
        if p_krit1(2)>=1
          p_c = q+u2;
        elseif p_krit1(2)<=0
          p_c = q;
        else
          p_c = p_krit1;
          switch ind_t2(3)
            case 1 % x=0
              p_c(1) = 0;
            case 2 % x=1
              p_c(1) = 1;
            case 3 % z=0
              p_c(3) = 0;
            case 4 % z=1
              p_c(3) = 1;
          end
          p_c = q+R_inv*p_c;
          d = norm(cross(p_c-p,u))/norm(u);
          pts = [p_c, [d; NaN; NaN]];
          pts = correct_pts(pts, u);
          return;
        end
        switch ind_t2(2)*ind_t2(3)
          case 3 % x=0,z=0
          case 4 % x=0,z=1
            p_c = p_c + u3;
          case 6 % x=1,z=0
            p_c = p_c + u1;
          case 8 % x=1,z=1
            p_c = p_c + u1 + u3;
          otherwise
            error(['Error, line parallel to one side intersects opposite '...
                   'sides after another.']);
        end
        d = norm(cross(p_c-p,u))/norm(u);
        pts = [p_c, [d; NaN; NaN]];
        pts = correct_pts(pts, u);
      else
        p_c = p_krit1;
        p_krit2 = pt+t2(i_krit+1)*ut;
        d = abs(max(min(p_c(2),0),p_c(2)-1))*norm(u2);
        p_c(2) = min(1,max(0,p_c(2)));
        pts = [q+R_inv*p_c, [d; norm(R_inv*p_krit1 - R_inv*p_krit2); NaN]];
        pts = correct_pts(pts, u);
      end
    elseif u3_parallel % g parallel to u3=const side only
      p_c = p_krit1([1 2]);
      if any(p_c>1+1e-10) || any(p_c<-1e-10) % nicht über Seite
        if p_krit1(3)>=1
          p_c = q+u3;
        elseif p_krit1(3)<=0
          p_c = q;
        else
          p_c = p_krit1;
          switch ind_t2(3)
            case 1 % x=0
              p_c(1) = 0;
            case 2 % x=1
              p_c(1) = 1;
            case 3 % y=0
              p_c(2) = 0;
            case 4 % y=1
              p_c(2) = 1;
          end
          p_c = q+R_inv*p_c;
          d = norm(cross(p_c-p,u))/norm(u);
          pts = [p_c, [d; NaN; NaN]];
          pts = correct_pts(pts, u);
          return;
        end
        switch ind_t2(2)*ind_t2(3)
          case 3 % x=0,y=0
          case 4 % x=0,y=1
            p_c = p_c + u2;
          case 6 % x=1,y=0
            p_c = p_c + u1;
          case 8 % x=1,y=1
            p_c = p_c + u1 + u2;
          otherwise
            error(['Error, line parallel to one side intersects opposite '...
                   'sides after another.']);
        end
        d = norm(cross(p_c-p,u))/norm(u);
        pts = [p_c, [d; NaN; NaN]];
        pts = correct_pts(pts, u);
      else
        p_c = p_krit1;
        p_krit2 = pt+t2(i_krit+1)*ut;
        d = abs(max(min(p_c(3),0),p_c(3)-1))*norm(u3);
        p_c(3) = min(1,max(0,p_c(3)));
        pts = [q+R_inv*p_c, [d; norm(R_inv*p_krit1 - R_inv*p_krit2); NaN]];
        pts = correct_pts(pts, u);
      end
    else
      sectors = NaN(7,3);
      krit_start = NaN;
      krit_end = NaN;
      for j=1:6
        i_sec = ceil(ind_t2(j)/2);
        if isnan(sectors(j+1,i_sec))
          sectors(j+1:7,i_sec) = 0;
          if ind_t2(j) == 2*i_sec-1
            sectors(1:j,i_sec) = -1;
          else
            sectors(1:j,i_sec) = 1;
          end
          if all(~isnan(sectors(j+1,:)))
            krit_end = j;
            break;
          end
        else
          if ind_t2(j) == 2*i_sec-1
            sectors(j+1:7,i_sec) = -1;
          else
            sectors(j+1:7,i_sec) = 1;
          end
          if isnan(krit_start)
            krit_start = j+1;
          end
        end
      end
      d_min = Inf;
      p_c = NaN(3,1);
      u_ges = [u1,u2,u3];
      for j=krit_start:krit_end
        c = q;
        for k=1:3
          if sectors(j,k)==1
            c = c + u_ges(:,k);
          end
        end
        switch sum(abs(sectors(j,:)))
          case 2
            if sectors(j,1)==0
              ui = u1;
            elseif sectors(j,2)==0
              ui = u2;
            else
              ui = u3;
            end
            cuiu = cross(ui,u);
            cucuiu = cross(u, cuiu);
            [d_min, p_c] = check_edge(c, ui, cuiu, cucuiu, p, p_c, d_min);
          case 3
            [d_min, p_c] = check_corner(c, p, u, p_c, d_min);
        end
      end
      pts = [p_c, [norm(cross(p_c-p,u))/norm(u); NaN(2,1)]];
      pts = correct_pts(pts, u);
    end
  else
    pts = [q+R_inv*p_krit1, q+R_inv*(pt+t2(i_krit+1)*ut)];
  end
end
  

% Hilfsfunktion, um zu überprüfen, ob eine Ecke den aktuellen
% Minmalabstand unterbietet
function [d_min, p_c] = check_corner(c, p, u, p_c, d_min)
  d = norm(cross(p-c,u))/norm(u);
  if d<d_min
    d_min = d;
    p_c = c;
  end
end

% Hilfsfunktion, um zu überprüfen, ob eine Kante den aktuellen
% Minmalabstand unterbietet
function [d_min, p_c] = check_edge(e, ui, cuiu, cucuiu, p, p_c, d_min)
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

% Hilfsfunktion zur Korrektur von Quaderschnittpunkten die nur numerisch
% außerhalb des Quaders liegen (d<1e-10)
function pts_out = correct_pts(pts_in, u)
  if isnan(pts_in(3,2)) && pts_in(1,2) < 1e-10
    if ~isnan(pts_in(2,2))
      pts_out = [pts_in(1:3,1) pts_in(1:3,1)+u*pts_in(2,2)];
    else
      pts_out = [pts_in(1:3,1) pts_in(1:3,1)];
    end
  else
    pts_out = pts_in;
  end
end
