% test intersection functions
rng(0);
t=-10:10;
% systematic test
p1 = [0 0 0]';
p2 = [0 0 3]';
r  = 2;
ps = [[0 0 0]' [0 0 0]' [0 0 3]' [0 0 1]' [0 0 -1]' [0 0 4]' [0 0 5]' [0 0 -2]' [2 0 1]' [0 0 6]' [0 0 -3]' [3 0 1]' [2 0 0]' [2 0 0]' [2 0 0]'];
us = [[0 0.1 1]' [0 1 1]' [0 1 1]' [0 1 0]' [0 1 0]' [0 1 0]' [0 1 0]' [0 1 0]' [0 1 0]' [0 1 0]' [0 1 0]' [0 1 0]' [-1 0 1]' [-1 0 0.5]' [-1 0 -1]'];
for i=1:length(ps(1,:))
  close(figure(1));
  close(figure(2));
  p   = ps(:,i);
  u   = us(:,i);
  v   = p2-p1;
  cuv = cross(u,v);
  q   = p+((p1-p).'*cross(v,cuv))/(cuv.'*cuv)*u;
  q1  = p+(p1-p).'*u/(u.'*u)*u;
  q2  = p+(p2-p).'*u/(u.'*u)*u;
  q3  = p1+(q-p1).'*v/(v.'*v)*v;
  e_z = (p2-p1)/norm(p2-p1);
  e_y = randi(10,3,1);
  while (e_y.'*e_z==0)
    e_y = randi(10,3,1);
  end
  e_y = e_y-e_y.'*e_z*e_z;
  e_y = e_y/norm(e_y);
  e_x = cross(e_y, e_z);
  R=[e_x e_y e_z]';
  pts1 = find_intersection_line_capsule(p,u,p1,p2,r);
  pts2 = find_intersection_line_cylinder(p,u,p1,p2,r);
  [c_x, c_y, c_z] = cylinder(r,20);
  c_z=c_z*norm(p2-p1);
  for j = 1:length(c_x(1,:))
    tmp = R'*[c_x(1,j); c_y(1,j); c_z(1,j)];
    c_x(1,j) = tmp(1);
    c_y(1,j) = tmp(2);
    c_z(1,j) = tmp(3);
    tmp = R'*[c_x(2,j); c_y(2,j); c_z(2,j)];
    c_x(2,j) = tmp(1);
    c_y(2,j) = tmp(2);
    c_z(2,j) = tmp(3);
  end
  [s_x, s_y, s_z] = sphere(20);
  figure(1);hold on;
  surf(c_x+p1(1), c_y+p1(2), c_z+p1(3),'facealpha',0.3);
  surf(s_x*r+p1(1), s_y*r+p1(2), s_z*r+p1(3),'facealpha',0.3);
  surf(s_x*r+p2(1), s_y*r+p2(2), s_z*r+p2(3),'facealpha',0.3);
  plot3([p(1)-100*u(1) p(1)+100*u(1)]', [p(2)-100*u(2) p(2)+100*u(2)]', [p(3)-100*u(3) p(3)+100*u(3)]','b');
  plot3([p1(1)-100*v(1) p1(1)+100*v(1)]', [p1(2)-100*v(2) p1(2)+100*v(2)]', [p1(3)-100*v(3) p1(3)+100*v(3)]', 'b');
  plot3([p1(1) p1(1)+v(1)]', [p1(2) p1(2)+v(2)]', [p1(3) p1(3)+v(3)]', 'r');
  plot3([p1(1) q1(1)]', [p1(2) q1(2)]', [p1(3) q1(3)]','y');
  plot3([p2(1) q2(1)]', [p2(2) q2(2)]', [p2(3) q2(3)]','g');
  plot3([q(1) q3(1)]', [q(2) q3(2)]', [q(3) q3(3)]','r');
  scatter3(pts1(1,:),pts1(2,:),pts1(3,:),50,'r','filled');
  xlim([-20,20]);
  ylim([-20,20]);
  zlim([-20,20]);
  axis equal;
  figure(2);hold on;
  surf(c_x+p1(1), c_y+p1(2), c_z+p1(3),'facealpha',0.3);
  plot3([p(1)-100*u(1) p(1)+100*u(1)]', [p(2)-100*u(2) p(2)+100*u(2)]', [p(3)-100*u(3) p(3)+100*u(3)]','b');
  plot3([p1(1)-100*v(1) p1(1)+100*v(1)]', [p1(2)-100*v(2) p1(2)+100*v(2)]', [p1(3)-100*v(3) p1(3)+100*v(3)]', 'b');
  plot3([p1(1) p1(1)+v(1)]', [p1(2) p1(2)+v(2)]', [p1(3) p1(3)+v(3)]', 'r');
  plot3([p1(1) q1(1)]', [p1(2) q1(2)]', [p1(3) q1(3)]','y');
  plot3([p2(1) q2(1)]', [p2(2) q2(2)]', [p2(3) q2(3)]','g');
  plot3([q(1) q3(1)]', [q(2) q3(2)]', [q(3) q3(3)]','r');
  plot3([p(1) pts2(1,1)]', [p(2) pts2(2,1)]', [p(3) pts2(3,1)]','m');
  scatter3(pts2(1,:),pts2(2,:),pts2(3,:),50,'r','filled');
  xlim([-20,20]);
  ylim([-20,20]);
  zlim([-20,20]);
  axis equal;
  pause;
end
% random test
for i=1:10
  close(figure(1));
  close(figure(2));
  p1  = randi(10,3,1);
  p2  = randi(10,3,1);
  p   = randi(10,3,1);
  u   = randi(10,3,1);
  v   = p2-p1;
  cuv = cross(u,v);
  q   = p+((p1-p).'*cross(v,cuv))/(cuv.'*cuv)*u;
  q1  = p+(p1-p).'*u/(u.'*u)*u;
  q2  = p+(p2-p).'*u/(u.'*u)*u;
  q3  = p1+(q-p1).'*v/(v.'*v)*v;
  r   = randi(4)+1;
  e_z = (p2-p1)/norm(p2-p1);
  e_y = randi(10,3,1);
  while (e_y.'*e_z==0)
    e_y = randi(10,3,1);
  end
  e_y = e_y-e_y.'*e_z*e_z;
  e_y = e_y/norm(e_y);
  e_x = cross(e_y, e_z);
  R=[e_x e_y e_z]';
  pts1 = find_intersection_line_capsule(p,u,p1,p2,r);
  pts2 = find_intersection_line_cylinder(p,u,p1,p2,r);
  [c_x, c_y, c_z] = cylinder(r,20);
  c_z=c_z*norm(p2-p1);
  for j = 1:length(c_x(1,:))
    tmp = R'*[c_x(1,j); c_y(1,j); c_z(1,j)];
    c_x(1,j) = tmp(1);
    c_y(1,j) = tmp(2);
    c_z(1,j) = tmp(3);
    tmp = R'*[c_x(2,j); c_y(2,j); c_z(2,j)];
    c_x(2,j) = tmp(1);
    c_y(2,j) = tmp(2);
    c_z(2,j) = tmp(3);
  end
  [s_x, s_y, s_z] = sphere(20);
  figure(1);hold on;
  surf(c_x+p1(1), c_y+p1(2), c_z+p1(3),'facealpha',0.3);
  surf(s_x*r+p1(1), s_y*r+p1(2), s_z*r+p1(3),'facealpha',0.3);
  surf(s_x*r+p2(1), s_y*r+p2(2), s_z*r+p2(3),'facealpha',0.3);
  plot3([p(1)-100*u(1) p(1)+100*u(1)]', [p(2)-100*u(2) p(2)+100*u(2)]', [p(3)-100*u(3) p(3)+100*u(3)]','b');
  plot3([p1(1)-100*v(1) p1(1)+100*v(1)]', [p1(2)-100*v(2) p1(2)+100*v(2)]', [p1(3)-100*v(3) p1(3)+100*v(3)]', 'b');
  plot3([p1(1) p1(1)+v(1)]', [p1(2) p1(2)+v(2)]', [p1(3) p1(3)+v(3)]', 'r');
  plot3([p1(1) q1(1)]', [p1(2) q1(2)]', [p1(3) q1(3)]','y');
  plot3([p2(1) q2(1)]', [p2(2) q2(2)]', [p2(3) q2(3)]','g');
  plot3([q(1) q3(1)]', [q(2) q3(2)]', [q(3) q3(3)]','r');
  scatter3(pts1(1,:),pts1(2,:),pts1(3,:),50,'r','filled');
  xlim([-20,20]);
  ylim([-20,20]);
  zlim([-20,20]);
  axis equal;
  figure(2);hold on;
  surf(c_x+p1(1), c_y+p1(2), c_z+p1(3),'facealpha',0.3);
  plot3([p(1)-100*u(1) p(1)+100*u(1)]', [p(2)-100*u(2) p(2)+100*u(2)]', [p(3)-100*u(3) p(3)+100*u(3)]','b');
  plot3([p1(1)-100*v(1) p1(1)+100*v(1)]', [p1(2)-100*v(2) p1(2)+100*v(2)]', [p1(3)-100*v(3) p1(3)+100*v(3)]', 'b');
  plot3([p1(1) p1(1)+v(1)]', [p1(2) p1(2)+v(2)]', [p1(3) p1(3)+v(3)]', 'r');
  plot3([p1(1) q1(1)]', [p1(2) q1(2)]', [p1(3) q1(3)]','y');
  plot3([p2(1) q2(1)]', [p2(2) q2(2)]', [p2(3) q2(3)]','g');
  plot3([q(1) q3(1)]', [q(2) q3(2)]', [q(3) q3(3)]','r');
  plot3([p(1) pts2(1,1)]', [p(2) pts2(2,1)]', [p(3) pts2(3,1)]','m');
  scatter3(pts2(1,:),pts2(2,:),pts2(3,:),50,'r','filled');
  xlim([-20,20]);
  ylim([-20,20]);
  zlim([-20,20]);
  axis equal;
  pause;
end