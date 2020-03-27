
%Load file with fitted line params
fid=fopen("line_params");
params=fread(fid,"double");

%read the angles corresponding to the line params
angles=params(3:4:end);
%calculated values of m from inclination angle 
tableM=(sin(angles));

%set up interpolation variable
table_ph=1:size(tableM);
table_ph=table_ph/size(tableM,1)*pi;
table_ph=transpose(table_ph);

%fit fourth order polygon
poly_coffs=polyfit(table_ph,tableM,4)
hold off
plot(table_ph,[tableM,polyval(poly_coffs,table_ph)]);
%create symbolic function from polynomial coefficients
rep_m=poly2sym(poly_coffs,ph)
fprintf("Pausing to look at the regression plot")
pause
%clear plot
newplot()

%d_smoothed=diff(smoothed);
%
%x=1:size(d_smoothed);
%d_smoothed=regdatasmooth(x,d_smoothed);
%
%
%
%
%dd_smoothed=diff(d_smoothed);
%
%x=1:size(dd_smoothed);
%dd_smoothed=regdatasmooth(x,dd_smoothed);
%
%
%
%ddd_smoothed=diff(dd_smoothed);
%
%x=1:size(ddd_smoothed);
%ddd_smoothed=regdatasmooth(x,ddd_smoothed);
%semilogy([smoothed(1:size(ddd_smoothed)),d_smoothed(1:size(ddd_smoothed)),dd_smoothed(1:size(ddd_smoothed)),ddd_smoothed])



