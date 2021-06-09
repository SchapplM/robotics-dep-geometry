% Calculate intersections of a line with a capsule or the nearest point, if
% no intersection exists
% 
% Input:
% p [3x1]
%   line base vector
% u [3x1]
%   line direction vector
% p1 [3x1]
%   center point of the first half-sphere of the capsule
% p2 [3x1]
%   center point of the second half-sphere of the capsule
% r scalar
%   radius of the capsule
% Output:
% pts [3x2]
%   intersection points, or nearest point on capsule and distance to
%   nearest point as well as line parameter until which the same distance is
%   kept if line is parallel to capsule axis stacked with NaNs, when no
%   intersections exist. The returned values are ordered such that the value
%   with the smaller s in x = p+s*u is returned in the first column. 

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-06
% (C) Institut für Regelungstechnik, Leibniz Universität Hannover

function pts = find_intersection_line_capsule(p, u, p1, p2, r)
%#codegen
%$cgargs {zeros(3,1),zeros(3,1),zeros(3,1),zeros(3,1),0}
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
  v = p2-p1;
  cuv = cross(u,v);
  s_c = NaN(6,1);
  % Fall 1: g parallel zum Mantel
  if norm(cuv)/(norm(u)*norm(v)) < 1e-10
    find_intersection_k1;
    if isnan(s_c(3))
      r1 = p-p1-(p-p1).'*u/(u.'*u)*u;
      if u.'*v < 0 % u entgegen v, auftreffpunkt auf Seite von p2
        p_c = p2+r1/norm(r1)*r;
      else % u in richtung v, Auftreffpunkt auf Seite von p1
        p_c = p1+r1/norm(r1)*r;
      end
      pts = [p_c, [norm(cross(p_c-p,u))/norm(u); norm(v)/norm(u); NaN]];
      correct_pts;
      return;
    else
      find_intersection_k2;
      if isnan(s_c(5))
        r1 = p-p1-(p-p1).'*u/(u.'*u)*u;
        if u.'*v < 0 % u entgegen v, auftreffpunkt auf Seite von p2
          p_c = p2+r1/norm(r1)*r;
        else % u in richtung v, Auftreffpunkt auf Seite von p1
          p_c = p1+r1/norm(r1)*r;
        end
        pts = [p_c, [norm(cross(p_c-p,u))/norm(u); norm(v)/norm(u); NaN]];
        correct_pts;
        return;
      end
    end
    pts = [p+min(s_c)*u, p+max(s_c)*u];
    return;
  end
  q = p+((p1-p).'*cross(v,cuv))/(cuv.'*cuv)*u;
  % Nun muss festgestellt werden, ob q innerhalb der Kapsel liegt. Dazu
  % berechnen wir die Schnittpunkte von g1 mit dem unendlichen Zylinder. Dazu
  % setzen wir
  % |(p+s*u-p1) x v|/|v| = r
  % <=> ((p-p1) x v+s*(u x v))*((p-p1) x v+s*(u x v))=r^2*v^2
  % <=> s^2*(u x v)^2+2*s*(u x v)*((p-p1) x v)+((p-p1) x v)^2-r^2*v^2=0
  % <=> s = -(u x v)*((p-p1) x v)/(u x v)^2+-sqrt((u x v)*((p-p1) x v)^2/(u x v)^4-((((p-p1) x v)^2-r^2*v^2)/(u x v)^2).
  % Liegen keine Schnittpunkte mit diesem vor (s nicht reell), so wird der
  % nächstgelegene Punkt zur Kapsel bestimmt. Wenn Schnittpunkte vorliegen,
  % wird geprüft, ob Sie auf dem zur Kapsel gehörigen Teil des unendlichen
  % Zylinders liegen. Das ist der Fall, wenn (q-p2)*v<0 und
  % (q-p1)*v>0 ist. Ist die erste Bedingung verletzt, gibt es
  % (mindestens) eine Schnittpunkt mit der Halbkugel um p1, ist die zweite
  % Bedingung verletzt, gibt es (mindestens) eine Schnittpunkt mit der
  % Halbkugel um p2. Dies folgt daraus, dass die Skalarprodukte die
  % relative Richtung der Projektion des Vektors von pi nach q auf den
  % Vektor von p1 nach p2 angibt. Für die beiden Kugeln wird, falls
  % notwendig der Schnittpunkt nach folgendem Ansatz berechnet:
  % (p-p1+s*u)^2 = r^2 <=> s^2*u^2+2*(p-p1)*u*s+(p-p1)^2-r^2=0
  % <=> s=-(p-p1)*u/u^2+-sqrt(((p-p1)*u)^2/u^4-((p-p1)^2-r^2)/u^2)
  % für die Halbkugel um p1 und
  % s=-(p-p2)*u/u^2+-sqrt(((p-p2)*u)^2/u^4-((p-p2)^2-r^2)/u^2)
  % für die Halbkugel um p2. Danach muss noch geprüft werden (wieder über
  % obiges Skalarprodukt), ob die gefundenen Schnittpunkte in der korrekten
  % Hälfte der Kugel liegen.

  cpp1v  = cross(p-p1,v);
  z = (cuv.'*cpp1v)^2/(cuv.'*cuv)^2-(cpp1v.'*cpp1v-r^2*(v.'*v))/(cuv.'*cuv);
  
  % Wenn keine Schnittpunkte vorliegen, muss der Punkt auf der Kapsel
  % berechnet werden, der der Geraden g1 am nächsten liegt. Dazu werden
  % wieder die drei Fälle "q jenseits von p1", "q jenseits von p2" und "q
  % zwischen p1 und p2" unterschieden. Fall 1 und 2: Addiert man zu p die
  % Projektion von des Vektors von p nach pi auf u, so erhält man den pi
  % nächstgelegenden Punkt qi auf der Geraden g1. Geht man von pi in Richtung
  % dieses Punktes (Weite r), so landet man am gesuchten Punkt p_c:
  % qi = p+(pi-p)*u/u^2*u, p_c = pi+(qi-pi)/|qi-pi|*r
  % Fall 3: wir kennen die Richtung von q nach g2: u x v. Den Lotfußpunkt
  % q3 auf g2 können wir analog zu Fall 1/2 berechnen:
  % q3 = p1+(q-p1)*v/v^2*v von da aus gehen wir um die Länge r in
  % Richtung (q-q3), um den gesuchten Punkt zu finden.
  if z < 0 % kein Schnittpunkt liegt vor, wähle nächsten Punkt
    pc1 = find_next_pkt;
    pts = [pc1, [norm(cross(pc1-p,u))/norm(u); NaN(2,1)]];
    correct_pts;
  else % Schnittpunkte gefunden, auf Korrektheit prüfen und ggf. korrigieren
    sqrt_z = sqrt(z);
    s_z = -cuv.'*cpp1v/(cuv.'*cuv);
    s_c(1)  = s_z-sqrt_z;
    s_c(2)  = s_z+sqrt_z;
    pc1 = p+s_c(1)*u;
    pc2 = p+s_c(2)*u;
    check_k1 = true;
    check_k2 = true;
    if (pc1-p1).'*v<-1e-10 % Schnittpunkt mit Halbkugel um p1 moeglich
      find_intersection_k1;
      check_k1 = false;
      s_c(1) = NaN;
    elseif (pc1-p2).'*v>1e-10 % Schnittpunkt mit Halbkugel um p2 moeglich
      find_intersection_k2;
      check_k2 = false;
      s_c(1) = NaN;
    end
    if (pc2-p1).'*v<-1e-10 % Schnittpunkt mit Halbkugel um p1 moeglich und noch nicht untersucht
      if check_k1
        find_intersection_k1;
        check_k1 = false;
      end
      s_c(2) = NaN;
    elseif (pc2-p2).'*v>1e-10 % Schnittpunkt mit Halbkugel um p2 moeglich und noch nicht untersucht
      if check_k2
        find_intersection_k2;
        check_k2 = false;
      end
      s_c(2) = NaN;
    end
    if (~check_k1 && isnan(s_c(3))) || (~check_k2 && isnan(s_c(5))) % Kapsel nicht geschnitten 
      pc1 = find_next_pkt;
      pts = [pc1, [norm(cross(pc1-p,u))/norm(u); NaN(2,1)]];
      correct_pts;
      return;
    end
    pts = [p+min(s_c)*u, p+max(s_c)*u];
  end
  
  % Hilfsfunktion zum Finden des nächstgelegenen Punktes auf der
  % Kapseloberfläche
  function p_out = find_next_pkt
    if (q-p1).'*v<0
      q1  = p+(p1-p).'*u/(u.'*u)*u;
      p_c = p1+(q1-p1)/norm(q1-p1)*r;
    elseif (q-p2).'*v>0
      q2  = p+(p2-p).'*u/(u.'*u)*u;
      p_c = p2+(q2-p2)/norm(q2-p2)*r;
    else
      q3  = p1+(q-p1).'*v/(v.'*v)*v;
      p_c = q3+(q-q3)/norm(q-q3)*r;
    end
    p_out=p_c;
  end

  % Hilfsfunktion zum Finden von SChnittpunkten mit der Kuppel um p1
  function find_intersection_k1
    s = ((p-p1).'*u)^2/(u.'*u)^2-((p-p1).'*(p-p1)-r^2)/(u.'*u);
    if s > 0 % falls keine Schnittpunkte vorliegen: nächster pkt
      sqrt_s = sqrt(s);
      s_s = -(p-p1).'*u/(u.'*u);
      s_c(3) = s_s-sqrt_s; %#ok<EMVDF>
      s_c(4) = s_s+sqrt_s; %#ok<EMVDF>
    end
  end
  % Hilfsfunktion zur Korrektheitsprüfung und Korrektur der
  % Zylinderschnittpunkte an der Seite p2
  function find_intersection_k2
    s = ((p-p2).'*u)^2/(u.'*u)^2-((p-p2).'*(p-p2)-r^2)/(u.'*u);
    if s > 0 % falls keine Schnittpunkte vorliegen: nächster pkt
      sqrt_s = sqrt(s);
      s_s = -(p-p2).'*u/(u.'*u);
      s_c(5) = s_s-sqrt_s; %#ok<EMVDF>
      s_c(6) = s_s+sqrt_s; %#ok<EMVDF>
    end
  end
  % Hilfsfunktion zur Korrektur von Kapselschnittpunkten die nur numerisch
  % außerhalb der Kapsel liegen (d<1e-10)
  function correct_pts
    if isnan(pts(3,2)) && pts(1,2) < 1e-10
      if ~isnan(pts(2,2))
        pts = [pts(1:3,1) pts(1:3,1)+u*pts(2,2)];
      else
        pts = [pts(1:3,1) pts(1:3,1)];
      end
    end
  end
end
  
