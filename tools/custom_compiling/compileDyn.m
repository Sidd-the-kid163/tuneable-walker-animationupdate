function [ ] = compileDyn( nlp, export_path )
%
% compile natural dynamics only: D matrix, C\dot{q},  G vectors
% ________________________________________________________

nDomain = floor(numel(nlp.Phase) / 2);

for ph_index = 1:2:nDomain

    ph_Constr = nlp.Phase(ph_index).ConstrTable;
    
    for i = 1 : length(ph_Constr.dynamics_equation(1).SummandFunctions)

        w = ph_Constr.dynamics_equation(1).SummandFunctions(i).Name;
        if startsWith(w, 'Mmat') || startsWith(w, 'Ce') || startsWith(w, 'Ge')
            export(ph_Constr.dynamics_equation(1).SummandFunctions(i).SymFun, export_path);
        end
        
    end

end

end