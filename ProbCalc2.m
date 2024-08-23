function p1 = ProbCalc2(m1,s1,m2,s2)
%% Prob. of being early. m1,s1 is request; m2,s2 is bid (mean, std. dev.).
%% E. Halbach   VTT   2023

mt = m2 - m1;
st = sqrt(s1^2 + s2^2);

p1 = normcdf(0,mt,st);
