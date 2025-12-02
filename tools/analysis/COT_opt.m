%%% To calculate COT of optimal slippery gait

%%% load the optimization results
listing = dir(['params/', behavior.name]);
gaitName = listing(end).name;
load(['params/', behavior.name, '/', gaitName, '/optData.mat']);
logOpt = data.logger;

%%% calculate power and velocity
P = []; %power
V = []; %velocity
for k = 1:2
    for i = 1: length(logOpt(k).flow.t)
        q = logOpt(k).flow.states.x(:,i);
        dq = logOpt(k).flow.states.dx(:,i);
        u = logOpt(k).flow.inputs.Control.u(:,i);
        
        tmp = com_velFun(q,dq);
        V(end+1) = tmp(1);
        
        tmp = dq(4:end).*u;
        P(end+1) = sum(abs(tmp));
    end
end

mg = 9.81 * sum([behavior.robotModel.Links(:).Mass]); 

%%% method 1: the power method
COT1 = mean(P) / (mg* mean(V))

%%% method2: the energy method
t = [logOpt(1).flow.t, logOpt(2).flow.t];
E = trapz(t,P);
q0 = logOpt(1).flow.states.x(:,1);
qf = logOpt(2).flow.states.x(:,end);
tmp = com_posFun(qf) - com_posFun(q0);
COT2 = E / (mg* tmp(1))


%% energy change
q_minus  = logOpt(2).flow.states.x(:,end);
dq_minus = logOpt(2).flow.states.dx(:,end);  
q_plus   = logOpt(1).flow.states.x(:,1);
dq_plus  = logOpt(1).flow.states.dx(:,1);

tmp = LeftMidFoot_velFun(q_minus, dq_minus);
fprintf('NSF pre-impact velocity: x: %.3f, z: %.3f\n\n', tmp(1), tmp(3))
tmp = RightMidFoot_velFun(q_plus, dq_plus);
fprintf('NSF pos-impact velocity: x: %.3f, z: %.3f\n\n', tmp(1), tmp(3))

com_minus = com_velFun(q_minus, dq_minus);
ke_minus = 0.5 * m * sum(com_minus.^2)
com_plus = com_velFun(q_plus,  dq_plus);
ke_plus = 0.5 * m * sum(com_plus.^2)
tmp = abs(ke_plus-ke_minus)/ke_minus;
fprintf('Kinetic energy lost is %f\n\n', tmp)