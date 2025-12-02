function [] = compileDynConstraints_woDCeG(nlp, export_path)
% 
% To compile dynamic constraints partially:
%     - compile everything EXCEPT D matrix, C\dot{q} & G vecotrs
%       such as springs, holonomic constraints
% _________________________________________________________________________

nDomain = numel(nlp.Phase);

for ph_index = 1:2:nDomain

    ph_Constr = nlp.Phase(ph_index).ConstrTable;
    
    for i = 1 : length(ph_Constr.dynamics_equation(1).SummandFunctions)

        w = ph_Constr.dynamics_equation(1).SummandFunctions(i).Name;
        if ~startsWith(w, 'Mmat') && ~startsWith(w, 'Ce') && ~startsWith(w, 'Ge')
            export(ph_Constr.dynamics_equation(1).SummandFunctions(i).SymFun, export_path);

            % If derivative level above 0 do jacobian
            if nlp.Options.DerivativeLevel > 0
                exportJacobian(ph_Constr.dynamics_equation(1).SummandFunctions(i).SymFun, export_path);
            end
            % If higher (max 2) then do hessian
            if nlp.Options.DerivativeLevel > 1
                exportHessian(ph_Constr.dynamics_equation(1).SummandFunctions(i).SymFun, export_path);
            end            
        end
        
    end

end

end