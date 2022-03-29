function J=MLC_evaluator_LQR(ind,parameters,i,fig)


sigma=parameters.problem_variables.sigma;
omega=parameters.problem_variables.omega;
Tf=parameters.problem_variables.Tf;
dt=parameters.problem_variables.dt;
R=parameters.problem_variables.R;
Tevmax=parameters.problem_variables.Tevmax;
x0=parameters.problem_variables.x0;

A=[sigma -omega
   omega sigma];

% Read LISP representation of the control law of the given individual
m=readmylisp_to_formal_MLC(ind);
% Replace the state variable in LISP from "S0, S1, ..." to "y(1), y(2), ..." to make it compatible with the control law implementation
m=strrep(m,'S0','y(1)');
m=strrep(m,'S1','y(2)');

% Wrap y as b
b=@(y)(y);
eval(['b=@(y)([0;' m ']);']);

% Cost function
f=@(t,y)([A*y(1:2)+b(y(1:2));y(1).^2+y(2).^2+R*sum(b(y).^2)+testt(toc,Tevmax)]);
J=parameters.badvalue;

try
tic
% Run simulation for the current individual
[T,Y]=ode45(f,[0:dt:Tf],[x0 ;0]');
if T(end)==Tf
    J=Y(end,3);
end
catch
   % Simulation crashed...
   fprintf('crashed\n');
end

% Plot the simulation result of current individual
if nargin>3
    figure(999)
    subplot(3,1,1)
    plot(T,Y(:,1:2),'linewidth',1.2)
    legend('a_1','a_2')
    ylabel('a_k');
    subplot(3,1,2)
    m=strrep(m,'y(1)','y(:,1)');
    m=strrep(m,'y(2)','y(:,2)');
    b=@(y)(y);
    eval(['b=@(y)(' m ');']);
    plot(T,b(Y(:,1:2)),'k','linewidth',1.2)
    ylabel('b')
    subplot(3,1,3)
    plot(T,Y(:,3),'k','linewidth',1.2)
    ylabel('$\int_0^t\frac{dJ}{dt}dt$','interpreter','latex')
    
end