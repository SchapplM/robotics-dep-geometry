% test intersection functions
rng(0);
t=-10:10;
% systematic test
q = [0 0 0]';
u1 = [1 0 0]';
u2 = [0 2 0]';
u3 = [0 0 3]';
% Testf�lle:
% zwei Seiten nebeneinander x-y
ps = [0.3 1.5 1]';% [0.3 0.5 1]' [0.7 0.5 1]' [0.7 1.5 1]'];
us = [1 1 0]';%     [1 -1 0]'    [1 1 0]'     [1 -1 0]'];
% zwei Seiten nebeneinander y-z
ps = [ps [0.5 1.5 2.5]'];% [0.5 0.5 2.5]' [0.5 0.5 0.5]' [0.5 1.5 0.5]'];
us = [us [0 1 -1]'];%      [0 1 1]'       [0 1 -1]'      [0 1 1]'];
% zwei Seiten nebeneinander x-z
ps = [ps [0.3 1 0.5]'];% [0.3 1 2.5]' [0.7 1 0.5]' [0.7 1 2.5]'];
us = [us [1 0 -1]'];%    [1 0 1]'     [1 0 1]'     [1 0 -1]'];
% zwei Seiten gegen�ber
ps = [ps [0.5 0.5 0.5]'];% [0.5 0.5 0.5]' [0.5 0.5 0.5]'];
us = [us [1 0 0]'];%       [0 1 0]'       [0 0 1]'];
% in Ebene durch Kante und Ecke
ps = [ps [0.5 0.5 0]'];
us = [us [1 1 0]'];
% in Ebene nur an Ecke
ps = [ps [0.5 -0.5 0]'];
us = [us [1 -1 0]'];
% nur durch Kante
ps = [ps [0.5 -0.5 1.5]'];
us = [us [1 -1 0]'];
% durch Kante und Seite
ps = [ps [-0.5 -0.5 1.5]'];
us = [us [1 1 0]'];
% entlang Kante
ps = [ps [0.5 0 0]'];
us = [us [1 0 0]'];
% Kein SP, n�chster Pkt an Kante
ps = [ps [0.4 -0.6 1.5]'];
us = [us [1 -1 0]'];
% Kein SP, n�chster Pkt an Ecke
ps = [ps [0.4 -0.6 0]'];
us = [us [1 -1 0]'];
for i=1:length(ps(1,:))
  close(figure(1));
  p   = ps(:,i);
  u   = us(:,i);
  pts = find_intersection_line_box(p, u, q, u1, u2, u3);
  p_goal = p+(pts(:,1)-p).'*u/(u.'*u)*u;
  figure(1);hold on;
  plot_cube2(q, u1, u2, u3, 'b');
  plot3([p(1)-10*u(1) p(1)+10*u(1)]', [p(2)-10*u(2) p(2)+10*u(2)]', [p(3)-10*u(3) p(3)+10*u(3)]','b');
  plot3([pts(1,1) p_goal(1)]', [pts(2,1) p_goal(2)]', [pts(3,1) p_goal(3)]','r');
  scatter3(pts(1,:),pts(2,:),pts(3,:),50,'r','filled');
  xlim([-20,20]);
  ylim([-20,20]);
  zlim([-20,20]);
  axis equal;
  title('Bitte die korrekte Erkennung des Schnittpunkts/ nächsten Punkts bestätigen [ENTER]');
  pause;
end
% random test
for i=1:10
  close(figure(1));
  p   = randi(10,3,1);
  u   = randi(10,3,1);
  q   = randi(10,3,1);
  u1  = randi(10,3,1);
  u2  = [0 0 0]';
  while (u1.'*u2==0)
    u2  = randi(10,3,1);
    u2  = u2-u1.'*u2/(u1.'*u1)*u1;
  end
  u3  = cross(u1,u2);
  pts = find_intersection_line_box(p,u,q,u1,u2,u3);
  p_goal = p+(pts(:,1)-p).'*u/(u.'*u)*u;
  figure(1);hold on;
  plot_cube2(q, u1, u2, u3, 'b');
  plot3([p(1)-10*u(1) p(1)+10*u(1)]', [p(2)-10*u(2) p(2)+10*u(2)]', [p(3)-10*u(3) p(3)+10*u(3)]','b');
  plot3([pts(1,1) p_goal(1)]', [pts(2,1) p_goal(2)]', [pts(3,1) p_goal(3)]','r');
  scatter3(pts(1,:),pts(2,:),pts(3,:),50,'r','filled');
  xlim([-20,20]);
  ylim([-20,20]);
  zlim([-20,20]);
  axis equal;
  title('Bitte die korrekte Erkennung des Schnittpunkts/ nächsten Punkts bestätigen [ENTER]');
  pause;
end