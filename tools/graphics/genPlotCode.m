function [] = genPlotCode(behavior, export_path)
% Description:
%  generate / compile some functionalities that are used ONLY for plotting.
% 
% _________________________________________________________________________

obj = behavior.robotModel;
x = obj.States.x;
dx = obj.States.dx;

com_SF = SymFunction('com_posFun', obj.com_fun, {x});
comV_SF = SymFunction('com_velFun', obj.comV_fun, {x, dx});
com_SF.export(export_path);
comV_SF.export(export_path);

% % obj.fs_fun.export(exportPath);
% if strcmp(obj.spring_conf, 'spring')
%     spring_func = SymFunction('springForce_plot', obj.fs_fun, {x,dx} );
%     spring_func.export(export_path);
% end

% obj.fs_fun.export(exportPath);
% spring_func = SymFunction('springForce_plot', obj.fs_fun, {x,dx} );
% spring_func.export(export_path);

%%% hip joint %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hip_pos = obj.getCartesianPosition(obj.Joints(obj.getJointIndices('RightHip')));
vhip = SymFunction('hipVelocity', jacobian(hip_pos, x) * dx, {x, dx});
vhip.export(export_path);



%%% feet %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
names = fields(obj.feet);
for i = 1:length(names)
    pos = SymFunction([names{i},'_posFun'], ...
                obj.feet.(names{i}).computeCartesianPosition, {x});
	pos.export(export_path);
    
    vel = SymFunction([names{i},'_velFun'], ...
          jacobian(obj.feet.(names{i}).computeCartesianPosition, x) * dx, ...
          {x, dx});
	vel.export(export_path);
end

end
