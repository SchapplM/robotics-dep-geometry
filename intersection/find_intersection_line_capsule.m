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
%   intersection points, or nearest point and distance to nearest point
%   stacked with NaNs, when no intersections exist

% Jonathan Vorndamme, vorndamme@irt.uni-hannover.de, 2016-06
% (C) Institut für Regelungstechnik, Leibniz Universität Hannover

function pts = find_intersection_line_capsule(p, u, p1, p2, r)
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
  
  % Hilfsfunktion zur Korrektheitsprüfung und Korrektur der
  % Zylinderschnittpunkte an der Seite p1
  function p_out = correct_intersection_k1
    sqrt_s = sqrt(((p-p1).'*u)^2/(u.'*u)^2-((p-p1).'*(p-p1)-r^2)/(u.'*u));
    if imag(sqrt_s)~=0 % falls keine Schnittpunkte vorliegen: nächster pkt
      p_out = find_next_pkt;
      pts = [p_out [norm(cross(p_out-p,u))/norm(u); NaN(2,1)]];
      finished=1;
      return;
    end
    s_s = -(p-p1).'*u/(u.'*u);
    s1 = s_s+sqrt_s;
    s2 = s_s-sqrt_s;
    pc11 = p+s1*u;
    pc12 = p+s2*u;
    if (pc11-p1).'*v<0 % pc11 auf richtiger Hälfte
      if (pc12-p1).'*v<0 % pc12 auf richtiger Hälfte
        pts=[pc11 pc12];
        finished=1;
        p_out=0;
        return;
      end
      p_out=pc11;
    elseif (pc12-p1).'*v<0 % pc12 auf richtiger Hälfte
      p_out=pc12;
    end
  end
  % Hilfsfunktion zur Korrektheitsprüfung und Korrektur der
  % Zylinderschnittpunkte an der Seite p2
  function p_out = correct_intersection_k2
    sqrt_s = sqrt(((p-p2).'*u)^2/(u.'*u)^2-((p-p2).'*(p-p2)-r^2)/(u.'*u));
    if imag(sqrt_s)~=0 % falls keine Schnittpunkte vorliegen: nächster pkt
      p_out = find_next_pkt;
      pts = [p_out [norm(cross(p_out-p,u))/norm(u); NaN(2,1)]];
      finished=1;
      return;
    end
    s_s = -(p-p2).'*u/(u.'*u);
    s1 = s_s+sqrt_s;
    s2 = s_s-sqrt_s;
    pc11 = p+s1*u;
    pc12 = p+s2*u;
    if (pc11-p2).'*v>0 % pc11 auf richtiger Hälfte
      if (pc12-p2).'*v>0 % pc12 auf richtiger Hälfte
        pts=[pc11 pc12];
        finished=1;
        p_out=0;
        return;
      end
      p_out=pc11;
    elseif (pc12-p2).'*v>0 % pc12 auf richtiger Hälfte
      p_out=pc12;
    end
  end

  cpp1v  = cross(p-p1,v);
  sqrt_z = sqrt((cuv.'*cpp1v)^2/(cuv.'*cuv)^2-(cpp1v.'*cpp1v-r^2*(v.'*v))/(cuv.'*cuv));
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
  if (imag(sqrt_z)~=0) % kein Schnittpunkt liegt vor, wähle nächsten Punkt
    pc1 = find_next_pkt;
    pts = [pc1, [norm(cross(pc1-p,u))/norm(u); NaN(2,1)]];
  else % Schnittpunkte gefunden, auf Korrektheit prüfen und ggf. korrigieren
    s_z = -cuv.'*cpp1v/(cuv.'*cuv);
    s1  = s_z+sqrt_z;
    s2  = s_z-sqrt_z;
    pc1 = p+s1*u;
    pc2 = p+s2*u;
    finished=0;
    if (pc1-p1).'*v<0 % Schnittpunkt mit Halbkugel um p1 liegt vor
      pc1 = correct_intersection_k1;
      if finished
        return;
      end
    elseif (pc1-p2).'*v>0 % Schnittpunkt mit Halbkugel um p2 liegt vor
      pc1 = correct_intersection_k2;
      if finished
        return;
      end
    end
    if (pc2-p1).'*v<0 % Schnittpunkt mit Halbkugel um p1 liegt vor
      pc2 = correct_intersection_k1;
    elseif (pc2-p2).'*v>0 % Schnittpunkt mit Halbkugel um p2 liegt vor
      pc2 = correct_intersection_k2;
    end
    pts=[pc1, pc2];
  end
end
  
