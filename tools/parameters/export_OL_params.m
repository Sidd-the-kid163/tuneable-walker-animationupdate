function params = export_OL_params(nlp, params, behavior, logger)
%
% fit states to a bezier curves for those open-loop domains
% _________________________________________________________________________

for i = 1: length(logger)
    
    VC_name = 'position_output_dynamics'; %-name of virtual constraints
    
    if ~any(contains(nlp.Phase(2*(i-1)+1).ConstrTable.Properties.VariableNames, VC_name))
        fprintf('*** domain %s is OPEN-loop, export its state trajectory.\n',...
                logger(i).plant.Name);
        
        q = logger(i).flow.states.x;
        qdot = logger(i).flow.states.dx;
        t = logger(i).flow.t;
        
        % recostruct tau
        RD2_vc = behavior.vertices.v1.VirtualConstraints.position;
        tau_func = RD2_vc.PhaseVariable;
        xv = behavior.robotModel.States.x;
        pposv = behavior.vertices.v1.Params.pposition;
        for j = 1:length(t)
            tmp = tau_func.subs(xv, logger(i).flow.states.x(:,j));
            tmp = tmp.subs(pposv, logger(i).static.params.pposition);
            tau(j) = double(tmp);
            % ppos = logger(i).static.params.pposition;
            % tau = (t- ppos(2))/(ppos(1)-ppos(2));
        end
        
        aposition_x = zeros(size(RD2_vc.OutputParams));
        for j = 4 : size(q,1)
            
            figIndex = 300+j; %%% modify to a nonzero value to see plot.
            
            clf(figIndex);
            % [fitresult, gof] = createFit_b5(tau, q(j,:), '', figIndex);
            [fitresult, gof] = createFit_b6(tau, q(j,:), '', figIndex);
            
            if gof.sse > 1e-5
                warning('qd(%d) fitting is bad: %s.\n', j, gof.sse); keyboard;
            end
            
            %%% use polyfit instead?
            % [p,S] = polyfit(tau, q(j,:), 5); 
            % [y_fit,delta] = polyval(p,tau,S);
            % figure(305); clf
            % plot(tau, q(j,:), 'o', tau, y_fit);
            
            for k = 1:size(aposition_x, 2)
                coe(k) = fitresult.(['a', num2str(k-1)]);
            end
            aposition_x(j-3,:) = coe;
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check the error in velocity terms 
            vel_a = qdot(j,:);
            a0 = coe(1); a1 = coe(2); a2 = coe(3);
            a3 = coe(4); a4 = coe(5); a5 = coe(6); a6 = coe(7);
            
            % velFunc = RD2_vc.DesiredFuncs{2};
            for k = 1:length(t)                
                x = tau(k); 
                Dy = 6*a6*x^5 - 6*a5*x^5 + 6*a0*(x - 1)^5 - 6*a1*(x - 1)^5 ...
                     - 30*a1*x*(x - 1)^4 + 30*a2*x*(x - 1)^4 - 30*a5*x^4*(x - 1) + 15*a4*x^4*(2*x - 2) ...
                     + 60*a2*x^2*(x - 1)^3 - 60*a3*x^2*(x - 1)^3 ...
                     - 60*a3*x^3*(x - 1)^2 + 60*a4*x^3*(x - 1)^2;
                
                taudot = RD2_vc.PhaseFuncs{2}.subs(behavior.robotModel.States.dx, qdot(:,k));
                taudot = double(taudot.subs(pposv,logger(i).static.params.pposition));
                
                vel_fit(k) = Dy*taudot;
            end
            h = figure(310+j); clf; title('Vel comparison'); h.WindowStyle = 'docked';
            plot(t, vel_a, 'o'); hold on; plot(t, vel_fit);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
        end
        
        % params{i}.aposition_x = aposition_x;
        % params{i}.aposition = aposition_x;
        tmp = aposition_x([4,3,1,2],:); % hack lol
        params{i}.aposition = tmp(:);
        
        fprintf('Open loop state trajectory fitted.\n')
        
    end
    
end


