%Init
warning('off', 'all');
%pkg load symbolic
%sympref reset
more off
%Declare variables
syms d_phi ph r yaw  d_yaw dd_yaw v_p x y z delta r0 real 
assumeAlso(r0 >0 )
assumeAlso(r>r0);
syms m(ph) real

%Planar velocity
v_p=1
%Cone coordinates
rep_x=r*cos(ph)
rep_y=r*sin(ph)
z=m*(r-r0)


%definitions of r and phi
rep_r=sqrt(x^2+y^2)
rep_ph=atan2(y,x)

%direction  derivates of x 
d_x=v_p *cos(yaw)
d_y=v_p *sin(yaw)

%derivatives 
d_ph=diff(rep_ph,x)*d_x+diff(rep_ph,y)*d_y
d_ph=simplify(subs(d_ph,[x,y],[rep_x,rep_y]))
d_r=diff(rep_r,x)*d_x+diff(rep_r,y)*d_y
d_r=simplify(subs(d_r,[x,y],[rep_x,rep_y]))




%calculate d_z
rep_z_xy=subs(z,[r,ph],[rep_r,rep_ph]) % z in terms of x and y
dx_z=diff(rep_z_xy,x) % derivative by x

dy_z=diff(rep_z_xy,y) % derivative by y
%Short forms (Derived by Hand)
dx_z=x/r*m-y*(r-r0)/r^2*diff(m)  
dy_z=y/r*m+x*(r-r0)/r^2*diff(m)

d_z=dx_z*d_x+dy_z*d_y  %derivative of z 
d_z=simplify(subs(d_z,[x,y],[rep_x,rep_y])) %Substitute in x(ph,r) y(ph,r)
temp_d_z=cos(ph-yaw)*m-(1-r0/r)*diff(m)*sin(ph-yaw) % Hand shortened expression
is_correct=simplify(d_z==temp_d_z,'Steps',50) % check if hand shortened and calculated are equal
d_z= temp_d_z %assign shorter version


f0=[d_r, d_ph ,0]
f1=[0 ,0 ,1]
l0h=dot(gradient(d_z,[r,ph,yaw]),f0)
l1h=dot(gradient(d_z,[r,ph,yaw]),f1)
l0l0h=dot(gradient(l0h,[r,ph,yaw]),f0)
l0l1h=dot(gradient(l1h,[r,ph,yaw]),f0)
l1l0h=dot(gradient(l0h,[r,ph,yaw]),f1)
l1l1h=dot(gradient(l1h,[r,ph,yaw]),f1)

OM=[d_z l0h l1h l0l0h l0l1h l1l0h l1l1h]