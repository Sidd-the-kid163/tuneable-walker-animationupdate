function [eigs] = eigen_value(hybrid_system, params)

ref = Ref();
ref.h = struct();
ref.h.calcs = {};
ref.h.calc = {};

x_plus = params{1}.x0;

x0 = x_plus(2:end);

tol = 1e-8;
options = optimoptions('fsolve','Display','iter',...
    'MaxFunEvals',2000,...
    'tolfun', tol,...
    'TolX',tol);


J = jacobian(x0, hybrid_system, ref);

eigs = abs(eig(J))
end

function [J] = jacobian(x, hybrid_system, ref)
% computes the Jacobian of a function
n = length(x);
fprintf('Simulating nominal: ');
fx = fsl(x,hybrid_system, ref);
eps=1.e-8; % could be made better
xperturb=x;
for i=1:n
    fprintf('Computing row %d of %d: ', i, n);
    xperturb(i)=x(i)+eps;
    % J(:,i)=(fsl(xperturb, hybrid_system, ref)-fx)/eps;
    J(:,i)=(fsl(xperturb, hybrid_system, ref)-fx)/eps;
    xperturb(i)=x(i);
end
end

function ret = fsl(x,hybrid_system,ref)

x0 = [0;x];

tic
logger = hybrid_system.simulate(0, x0, [], [],'NumCycle', 1);
toc

guard = hybrid_system.Gamma.Edges.Guard{end};

% Compute reset map at the guard
xm = [logger(end).flow.states.x(:,end); logger(end).flow.states.dx(:,end)];
[~, x_f, ~]  = guard.calcDiscreteMap(0, xm);
    
ret = x_f(2:end);
end