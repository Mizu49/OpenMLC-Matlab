function J = MLC_evaluator_springmass(ind,parameters,i,fig)

%% Parameters
% Parameters of the spring-mass system
mass = parameters.problem_variables.mass;
stiffness = parameters.problem_variables.stiffness;
damping = parameters.problem_variables.damping;

% Time length of the simulation
Tf=parameters.problem_variables.Tf;
% Sampling time of the simulation
dt=parameters.problem_variables.dt;
% Weight matrix for the input vector in cost function
R=parameters.problem_variables.R;
% time out for the simulation (t > Tevmax). see `testt.m` 
Tevmax=parameters.problem_variables.Tevmax;
% Initial value of the state vector
x0=parameters.problem_variables.x0;

% System matrix
sysA = [0 1
   -stiffness/mass -damping/mass];
% Control input matrix
sysB = [
    0
    1/mass
];

%% Formulate control law
% Read LISP representation of the control law of the given individual
m=readmylisp_to_formal_MLC(ind);
% Replace the state variable in LISP from "S0, S1, ..." to "y(1), y(2), ..." to make it compatible with the control law implementation
m=strrep(m,'S0','y(1)');
m=strrep(m,'S1','y(2)');

% Define control law
% Converting string representation into MATLAB program
eval(['b=@(y)(' m ');']);

% Cost function for `ode45`
f=@(t,y)([
    sysA*y(1:2) + sysB * b(y(1:2)); % Time evolution of the system
    y(1).^2+y(2).^2+R*sum(b(y).^2) + testt(toc,Tevmax) % Cost function to be evaluated `testt()` is for timeout
]);
J=parameters.badvalue;

%% Simulation
try
tic
% Run simulation for the current individual
[T,Y]=ode45(f, 0:dt:Tf, [x0 ;0]');

if T(end)==Tf
    J=Y(end,3);
end
catch
   % Simulation crashed...
   fprintf('crashed\n');
end

%% Plot the simulation result of current individual
if nargin>3
    figure
    plot(T,Y(:,1),'linewidth',1.2)
    legend('a_1')
    ylabel('a_k');
    figure;
    m=strrep(m,'y(1)','y(:,1)');
    m=strrep(m,'y(2)','y(:,2)');
    b=@(y)(y);
    eval(['b=@(y)(' m ');']);
    plot(T,b(Y(:,1:2)),'k','linewidth',1.2)
    ylabel('b')
    figure;
    plot(T,Y(:,3),'k','linewidth',1.2)
    ylabel('$\int_0^t\frac{dJ}{dt}dt$','interpreter','latex')
    
end