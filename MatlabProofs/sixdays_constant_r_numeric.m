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
d_x=v_p*cos(yaw)
d_y=v_p*sin(yaw)

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



dd_z=simplify(diff(d_z,ph)*d_ph+diff(d_z,yaw)*d_yaw+diff(d_z,r)*d_r, 'IgnoreAnalyticConstraints', true, 'Steps',10) %derivative of d_z
temp_dd_z=d_yaw*(sin(ph-yaw)*m+(1-r0/r)*diff(m)*cos(ph-yaw))+sin(ph-yaw)^2/r*(m+diff(m,2)*(1-r0/r))-2*r0/r^2*cos(ph-yaw)*sin(ph-yaw)*diff(m) % Hand shortened expressio
is_correct=simplify(dd_z==temp_dd_z,'Steps',50) %check if hand shortened and calculated are equal
dd_z=temp_dd_z %assign shorter version

ddd_z=simplify(diff(dd_z,ph)*d_ph+diff(dd_z,yaw)*d_yaw+diff(dd_z,r)*d_r+diff(dd_z,d_yaw)*dd_yaw, 'IgnoreAnalyticConstraints', true, 'Steps',10) %derivative of dd_z
%Try to sort ddd_z so that it can be read better
ddd_z=collect(collect(collect(ddd_z,[d_yaw,dd_yaw]),[cos(ph-yaw),sin(ph-yaw),cos(2*ph-2*yaw),sin(2*ph-2*yaw),cos(3*ph-3*yaw),sin(3*ph-3*yaw)]),[m,diff(m),diff(m,2),diff(m,3),diff(m,4)])


%Measurement Equations
MeasEqs=[d_z,dd_z,ddd_z]
%Calculate Jacobian w.r.t. ph,r, delta
J=simplify(expand(jacobian(MeasEqs,[ph,r,yaw])), 'IgnoreAnalyticConstraints', true, 'Steps', 50)
%substitute values derived from the assumption r=r0
J=subs(J,[yaw, d_yaw,dd_yaw],[ph+pi/2,1/r,0])
%Read Sixdays track height function m(phi) from file
create_track_function
J=subs(J,[m,r0],[rep_m,10])
f=det(J)

 hold on
for r_value=10:0.1:11
f_temp=subs(f,r,r_value)
fplot(f_temp,[0, pi])
end
