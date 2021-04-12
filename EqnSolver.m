% equations solver for MatLab  Use S.'variable name' to call individual
% solutions after solving as needed.

% Use sigfig to adjust the number of sigfigs displayed in solutions
sigfig = 6;

%Enter all of the variables involved here on both lines:
syms P N       %Define symbolic variables
vars = [P N]; %Enclose them in a vector

%Fill in set of equations separated by commas, use == for equals
g=9.81;
m = 1.6;
theta = 50*(pi/180);
vo = 1.3;
ao= 1.4;
r = .6*exp(theta)
vr = r*vo
ar = r*(vo^2+ao)
Ar = ar-r*vo^2
Ao = r*ao + 2*vr*vo
psi = pi/4;
S=8.4;
W=m*g;


%Here are the equations seperated by semicolons:
eqns = [m*Ar == N*sin(psi) - W*sin(theta) - S*sin(psi);
    m*Ao == P - N*sin(psi) - S*sin(psi) - W*cos(theta)]

%Solve the system
S = vpasolve(eqns,vars);

%convert the solutions to cell data
c=struct2cell(S);
solns=cat(1,c{:});

%intiate precision of sigfig and display answers, then revert to previous
% standard value for vpa display
digitsOld = digits(6);
[vars;solns']'
digits(digitsOld);
     
     
% Use S.(variable) to call specific variables:
% For example S.fa or S.fb to call fa or fb solution
     

% 
% %fill ur variables here
% syms fa fb fc nb na wb nc
% vars = [fa fb fc nb na wb nc];
% 
% %fill out ur equations here
% eqns = [fa+fb*sind(30)-nb*cosd(30)==0,
%     na-8-fb*cosd(30)-nb*sind(30)==0,
%     fa==fb,
%     nb*cosd(30)-fb*sind(30)-nc==0,
%     nb*sind(30)+fb*cosd(30)-wb+fc==0,
%     fc==.5*nc,
%     fa==.5*na];
% 
% 
% S = vpasolve(eqns,vars);
% %Output your answers
% 
% c=struct2cell(S);
%      m=cat(1,c{:});
%      solns = [vars;m']'
% 
% % Use S.(variable) to call specific variables