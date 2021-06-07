% Calculate intersections of a line with a cylinder or the nearest point on
% the zylinder, if no intersection exists
% 
% Input:
% p [3x1]
%   line base vector
% u [3x1]
%   line direction vector
% p1 [3x1]
%   center point of the first end of the cylinder
% p2 [3x1]
%   center point of the second end of the cylinder
% r scalar
%   radius of the cylinder
% Output:
% pts [3x2]
%   intersection points, or nearest point on cylinder and distance to
%   nearest point as well as line parameter until which the same distance is
%   kept if line is parallel to zylinder axis or side planes stacked with NaNs,
%   when no intersections exist. The returned values are ordered such that the
%   value with the smaller s in x = p+s*u is returned in the first column.

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-06
% (c) Institut für Regelungstechnik, Universität Hannover

function pts = find_intersection_line_cylinder(p, u, p1, p2, r)
  %#codegen
  assert(isa(p,'double') && isreal(p) && all(size(p) == [3 1]), ...
    'find_intersection_line_cylinder: p has to be [3x1] double');
  assert(isa(u,'double') && isreal(u) && all(size(u) == [3 1]), ...
    'find_intersection_line_cylinder: u has to be [3x1] double');
  assert(isa(p1,'double') && isreal(p1) && all(size(p1) == [3 1]), ...
    'find_intersection_line_cylinder: p1 has to be [3x1] double');
  assert(isa(p2,'double') && isreal(p2) && all(size(p2) == [3 1]), ...
    'find_intersection_line_cylinder: p2 has to be [3x1] double');
  assert(isa(r,'double') && isreal(r) && all(size(r) == [1 1]), ...
    'find_intersection_line_cylinder: r has to be [3x1] double');
  
  % Die Eingaben definieren den Zylinder durch die Mittelpunkte der Deckel p1
  % und p2 sowie den Radius r und die Gerade g: x = p+s*u. Die Zylinderachse
  % ergibt sich daraus zu v = p2-p1.
  % Grundsätzlich werden drei Fälle unterschieden: 
  % 1. v parallel zu u: Hier gibt es drei Fälle: die Deckel werden geschnitten
  % (Ergebnis sind zwei Schnittpunkte mit den Deckeln), der Mantel wird tangiert
  % (Ergebnis sind zwei Schnittpunkte mit den Deckeln), die geraden schneidet
  % den Zylinder nicht (Ergebnis ist der Schnittpunkt mit der Deckelfläche mit
  % dem niedrigeren s-Parameter in der Geradegleichung x = p+s*u, sowie der
  % Abstand zum Mantel und die Länge des Zylinders).
  % Für die Fälle 2 und 3 wird zunächst der Punkt q auf der geraden g berechnet
  % der der Zylinderachse am nächsten ist.
  % 2. v parallel zu den Deckelflächen: zunächst wird unterschieden ob der
  % Abstand von zur Zylinderachse größer ist als r (keine Schnittpunkte, der
  % Nächste Punkt ist entweder der Punkt auf dem Weg von q zur Zylinderachse mit
  % Abstand r zur Zylinderachse (wenn q zwischen den Deckelflächen liegt) oder
  % die Projektion auf die Deckelfläche auf deren Seite die Gerade g liegt. Ist
  % der Abstand von q zur Zylinderachse kleiner als r wird unterschieden, ob q
  % zwischen den Deckelflächen liegt (Ergebnis zwei Schnittpunkte) oder
  % Ausserhalb (Ergebnis Projektion des Schnittpunktes von g mit dem unendlichen
  % Zylinder in die Deckelfläche uf deren Seite g verläuft, Abstand zum Deckel
  % und die Länge vom 1. bis zum 2. Schnittpunkt mit dem unendlichen Zylinder.
  % 3. v windschief zu u: Zunächst wird wieder unterschieden, ob der Abstand von
  % q zur Zylinderachse größer als r ist. In diesem Fall wird der nächste Punkt
  % numerisch bestimmt nach dem Verfahren des goldenen Schnitts. Anderenfalls
  % wird geschaut ob die Gerade g den Zylinder schneidet. Das ist der Fall, wenn
  % entweder q zwischen den Deckelflächen liegt oder q jenseits einer der
  % Deckelflächen liegt, aber der Schnittpunkt von g mit den Deckelflächen
  % innerhalb des Deckels liegt. Liegen CShnittpunkte vor werden die
  % Cshnittpunkte von g mit dem unendlichen Zylinder und den Deckelflächen
  % berechnet und nach ihren s-Parameetern sortiert. Die mittleren beiden Punkte
  % sind dann das Resultat. Liegen keine Schnittpunkte vor wird erneut das
  % Verfahren nach dem goldenen Schnitt bemüht.
  
  v = p2-p1;
  cuv = cross(u,v);
  % Fall 1: g parallel zum Mantel
  if norm(cuv)/(norm(u)*norm(v)) < 1e-10
    r1 = p-p1-(p-p1).'*u/(u.'*u)*u;
    if norm(r1) > r*(1+1e-10)
      if u.'*v < 0 % u entgegen v, auftreffpunkt auf Seite von p2
        p_c = p2+r1/norm(r1)*r;
      else % u in richtung v, Auftreffpunkt auf Seite von p1
        p_c = p1+r1/norm(r1)*r;
      end
      pts = [p_c, [norm(cross(p_c-p,u))/norm(u); norm(v)/norm(u); NaN]];
      pts = correct_pts(pts, u);
      return;
    else % Schnittpunkte auf beiden Zylinderflächen
      if u.'*v < 0 % u entgegen v, auftreffpunkt auf Seite von p2
        pts = [p2+r1 p1+r1];
      else % u in richtung v, Auftreffpunkt auf Seite von p1
        pts = [p1+r1 p2+r1];
      end
      return;
    end
  end

  % Berechne Lotfußpunkt q der windschiefen geraden g1: x=p+s*u; s aus R und
  % g2: x=p1+t*v; v=p2-p1, t aus R auf der geraden g1. Dazu wird die
  % Hilfsebene E: x=p1+s*v+t*(u x v) konstruiert, die g2 enthält und als
  % zweite Richtung die senkrechte zu beiden geraden (u x v) enthält. Der
  % normalenvektor von E lautet demnach v x (u x v). Die Normalengleichung
  % ergibt sich als (x-p1)*(v x (u x v))=0. Setzt man die geradengleichung
  % von g1 hier ein, kann man den parameter s für den Lotfußpunkt
  % bestimmen: (p+s*u-p1)*(v x (u x v))=0
  % <=> s*u*(v x (u x v))=(p1-p)*(v x (u x v))
  % <=> s=((p1-p)*(v x (u x v)))/(u*(v x (u x v)))
  % <=> s=((p1-p)*(v x (u x v)))/((u x v)*(u x v)), da a*(b x c)=(a x b)*c
  % <=> s=((p1-p)*(v x (u x v)))/|u x v|^2
  % daraus folgt q = p+s*u=p+((p1-p)*(v x (u x v)))/|u x v|^2*u.
  q = p+((p1-p).'*cross(v,cuv))/(cuv.'*cuv)*u;
  % Nun muss festgestellt werden, ob q innerhalb des Zylinders liegt. Dazu
  % berechnen wir die Schnittpunkte von g1 mit dem unendlichen Zylinder. Dazu
  % setzen wir
  % |(p+s*u-p1) x v|/|v| = r
  % <=> ((p-p1) x v+s*(u x v))*((p-p1) x v+s*(u x v))=r^2*v^2
  % <=> s^2*(u x v)^2+2*s*(u x v)*((p-p1) x v)+((p-p1) x v)^2-r^2*v^2=0
  % <=> s = -(u x v)*((p-p1) x v)/(u x v)^2+-sqrt((u x v)*((p-p1) x v)^2/(u x v)^4-((((p-p1) x v)^2-r^2*v^2)/(u x v)^2).
  % Liegen keine Schnittpunkte mit diesem vor (s nicht reell), so wird der
  % nächstgelegene Punkt zum Zylinder bestimmt. Wenn Schnittpunkte vorliegen,
  % wird geprüft, ob Sie auf dem zum Zylinder gehörigen Teil des unendlichen
  % Zylinders liegen. Das ist der Fall, wenn (q-p2)*v<0 und
  % (q-p1)*v>0 ist. Ist die erste Bedingung verletzt, gibt es
  % (mindestens) einen Schnittpunkt mit der Randfläche um p1, ist die zweite
  % Bedingung verletzt, gibt es (mindestens) eine Schnittpunkt mit der
  % Randfläche um p2. Dies folgt daraus, dass die Skalarprodukte die
  % relative Richtung der Projektion des Vektors von pi nach q auf den
  % Vektor von p1 nach p2 angibt. Für die beiden Randflächen wird, falls
  % notwendig der Schnittpunkt nach folgendem Ansatz berechnet: zuerst wird
  % der Schnittpunkt qi von g1 mit der Ebene E_i: (x-pi)*v=0 berechnet
  % und anschließend geprüft, ob |qi-pi|<r gilt.
  % (p+s*u-p1)*v=0 <=> s = (p1-p)*v/(u*v)
  % für die Randfläche um p1 und
  % s = (p2-p)*v/(u*v)
  % für die Randfläche um p2.
  
  cpp1v  = cross(p-p1,v);
  % Wenn keine Schnittpunkte vorliegen, muss der Punkt auf dem Zylinder
  % berechnet werden, der der Geraden g1 am nächsten liegt. Dazu werden
  % wieder die drei Fälle "q jenseits von p1", "q jenseits von p2" und "q
  % zwischen p1 und p2" unterschieden. Fall 1 und 2: lässt sich nur durch
  % eine Minimierungsaufgabe lösen. Dazu nutzen wir das Verfahren nach
  % dem goldenen Schnitt.
  % Fall 3: wir kennen die Richtung von q nach g2: u x v. Den Lotfußpunkt
  % q3 auf g2 können wir durch eine Projektion von q-p1 auf v berechnen:
  % q3 = p1+(q-p1)*v/v^2*v von da aus gehen wir um die Länge r in
  % Richtung (q-q3), um den gesuchten Punkt zu finden.
  
  radikant = (cuv.'*cpp1v)^2/(cuv.'*cuv)^2-(cpp1v.'*cpp1v-r^2*(v.'*v))/(cuv.'*cuv);
  
  % Fall 2: g parallel zum Deckel
  if abs(u.'*v/(norm(u)*norm(v))) < 1e-10
    if radikant < 0
      q3 = p1+(q-p1).'*v/(v.'*v)*v;
      q3q = q-q3;
      if (q-p1).'*v < 0 % q jenseits von p1
        p_c = p1 + q3q/norm(q3q)*r;
      elseif (q-p2).'*v > 0 % q jenseits von p2
        p_c = p2 + q3q/norm(q3q)*r;
      else % q zwischen p1 und p2
        p_c = q3 + q3q/norm(q3q)*r;
      end
      pts = [p_c [norm(cross(p_c-p,u))/norm(u); 0; NaN]];
      pts = correct_pts(pts, u);
    else
      sqrt_z = sqrt(radikant);
      s_z = -cuv.'*cpp1v/(cuv.'*cuv);
      s1  = s_z+sqrt_z;
      s2  = s_z-sqrt_z;
      pc2 = p + s2*u;
      if (q-p1).'*v < 0 % q jenseits von p1
        p_c = pc2+(p1-pc2).'*v/(v.'*v)*v;
      elseif (q-p2).'*v > 0 % q jenseits von p2
        p_c = pc2+(p2-pc2).'*v/(v.'*v)*v;
      else % q zwischen p1 und p2
        pts = [pc2 p+s1*u];
        return;
      end
      pts = [p_c [norm(cross(p_c-p,u))/norm(u); 2*sqrt_z; NaN]];
      pts = correct_pts(pts, u);
    end
    return;
  end
  
  % Fall 3: g windschief zur Zylinderachse
  if radikant < 0
    % kein Schnittpunkt liegt vor, wähle nächsten Punkt
    pc1 = find_next_pkt(q,p1,p2,p,v,u,r);
    pts = [pc1, [norm(cross(pc1-p,u))/norm(u); NaN(2,1)]];
    pts = correct_pts(pts, u);
    return;
  else % Schnittpunkte moeglich
    sqrt_z = sqrt(radikant);
    s_z = -cuv.'*cpp1v/(cuv.'*cuv);
    s1  = s_z+sqrt_z;
    s2  = s_z-sqrt_z;
    s3  = (p1-p).'*v/(u.'*v);
    s4  = (p2-p).'*v/(u.'*v);
    if (q-p1).'*v < 0 % q jenseits von p1
      p_c = p + s3*u;
      if norm(p_c-p1) > r % kein Schnittpunkt
        p_c = find_next_pkt(q,p1,p2,p,v,u,r);
        pts = [p_c, [norm(cross(p_c-p,u))/norm(u); NaN(2,1)]];
        pts = correct_pts(pts, u);
        return;
      end
    end
    if (q-p2).'*v > 0 % q jenseits von p2
      p_c = p + s4*u;
      if norm(p_c-p2) > r % kein Schnittpunkt
        p_c = find_next_pkt(q,p1,p2,p,v,u,r);
        pts = [p_c, [norm(cross(p_c-p,u))/norm(u); NaN(2,1)]];
        pts = correct_pts(pts, u);
        return;
      end
    end
    % Die relevanten Schnittpunkte sind immer die mittleren beiden
    s = sort([s1,s2,s3,s4]);
    pts = [p + s(2)*u, p + s(3)*u];
  end
end

% Hilfsfunktion zum Finden des nächstgelegenen Punktes auf der
% Zylinderoberfläche
% Input:
% q [3x1]
%   Lotfußpunkt auf der Geraden
% p1 [3x1]
%   center point of the first end of the cylinder
% p2 [3x1]
%   center point of the second end of the cylinder
% p [3x1]
%   line base vector
% v [3x1]
%   direction vector of the cylinder axis
% u [3x1]
%   line direction vector
% r scalar
%   radius of the cylinder
% 
% Output:
% p_out
%   Nächster Punkt auf dem Zylinder
function p_out = find_next_pkt(q,p1,p2,p,v,u,r)
  if (q-p1).'*v < 0 % q jenseits von p1
    p_in = p+(p1-p).'*v/(u.'*v)*u;
    p_c = golden_section_search(q, p_in, p1, v, r);
    p_c_p = p_c+(p1-p_c).'*v/(v.'*v)*v;
    p_c_pp1 = p_c_p-p1;
    p_c = p1+(p_c_pp1)/norm(p_c_pp1)*r;
  elseif (q-p2).'*v > 0 % q jenseits von p2
    p_in = p+(p2-p).'*v/(u.'*v)*u;
    p_c = golden_section_search(q, p_in, p2, v, r);
    p_c_p = p_c+(p2-p_c).'*v/(v.'*v)*v;
    p_c_pp2 = p_c_p-p2;
    p_c = p2+(p_c_pp2)/norm(p_c_pp2)*r;
  else % q zwischen p1 und p2
    q3  = p1+(q-p1).'*v/(v.'*v)*v;
    p_c = q3+(q-q3)/norm(q-q3)*r;
  end
  p_out = p_c;
end

% Hilfsfunktion zur Berechnung des Abstandes eines Punktes zu einer
% Kreisscheibe. Die Idee ist, zunächst den Abstand des Punktes zur
% Kreisebene l1 zu berechnen und anschließend den Abstand in der Ebene l2
% und beide mittels Pythagoras zu kombinieren. Eingaben: p Punkt, m
% Mittelpunkt des Kreises, n Normalenvektor und r Radius
function d = get_distance(p, m, n, r)
  pm = p-m;
  l1 = (pm)'*n;
  l2 = max(norm(pm-l1*n)-r,0);
  d = l1^2+l2^2;
end

% Hilfsunktion zur Suche des naechstegelegenen Punktes auf der geraden mit
% dem goldenen Schnitt. Eingabe zwei Punkte p1 und p2 zwischen denen
% gesucht wird, m, n, r 
function p_out = golden_section_search(p1, p2, m, n, r)
  n=n./norm(n);
  gs = (sqrt(5)-1)/2;
  p12=p2-p1;
  lambda1 = 0;
  lambda4 = 1;
  d1 = get_distance(p1+lambda1*p12, m, n, r);
  d4 = get_distance(p1+lambda4*p12, m, n, r);
  lambda2 = gs*lambda1+(1-gs)*lambda4;
  lambda3 = (1-gs)*lambda1+gs*lambda4;
  d2 = get_distance(p1+lambda2*p12, m, n, r);
  d3 = get_distance(p1+lambda3*p12, m, n, r);
  if d1<d2 && d1<d3 && d4<d2 && d4<d3
    error('Minimum nicht im gewaehlten Bereich oder Funktion nicht unimodal');
  end
  while abs(lambda1-lambda4)>1e-12
    if d2<d3
      lambda4 = lambda3;
      lambda3 = lambda2;
      lambda2 = gs*lambda1+(1-gs)*lambda4;
      d3 = d2;
      d2 = get_distance(p1+lambda2*p12, m, n, r);
    else
      lambda1 = lambda2;
      lambda2 = lambda3;
      lambda3 = (1-gs)*lambda1+gs*lambda4;
      d2 = d3;
      d3 = get_distance(p1+lambda3*p12, m, n, r);
    end
  end
  if d2<d3
    p_out = p1+lambda2*p12;
  else
    p_out = p1+lambda3*p12;
  end
end

% Hilfsfunktion zur Korrektur von Zylinderschnittpunkten die nur numerisch
% außerhalb des Zylinders liegen (d<1e-10)
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
