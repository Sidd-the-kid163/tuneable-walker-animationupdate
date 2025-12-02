function MakeGIFOG(conGUI,logger,folder)

delay_time = 1/15; %the delay time in seconds between frames

repeat_times = 5;

t = []; tlast = 0;
q = []; xoffset = 0;
for i = 1:repeat_times
    t = [t, logger(1).flow.t + tlast]; %#ok<*AGROW>
    tlast = t(end);
    xstep = logger(1).flow.states.x;
    xstep(1,:) = xstep(1,:) + xoffset;
    q = [q, xstep];
    xoffset = q(1,end)-q(1,1);
end
    
% get correct resolution for animation:
des_t = 0:delay_time:t(end);
diff_t = abs(repmat(t,length(des_t),1)-des_t');
[~,des_inds] = min(diff_t,[],2);

if ~isfolder(folder)
    mkdir(folder)
end
    
    gif(fullfile(folder,'sim.gif'),'DelayTime',delay_time, 'overwrite', true);
    for i = des_inds'
        conGUI.anim.Draw(t(i),q(:,i))
        gif
    end
    
    % Also save 8 gait tiles over one step
    % plotGaitTiles(conGUI.anim,logger,folder);

end