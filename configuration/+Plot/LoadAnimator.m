function [conGUI] = LoadAnimator(behavior, logger, speed, varargin)
    
    speed(isempty(speed)) = 0.5;
    
    if isa(logger, 'SimLogger')
        np = length(logger);
        
        t = [];
        q = [];
        for i=1:np
            t = [t, logger(i).flow.t];
            q = [q, logger(i).flow.states.x];
        end
    else
        np = length(logger);
        
        t = [];
        q = [];
        for i=1:np
            t = [t, logger(i).tspan];
            q = [q, logger(i).states.x];
        end
    end

    robot_disp = Plot.LoadDisplay(behavior, varargin{:});
    
    anim = frost.Animator.AbstractAnimator(robot_disp, t, q);
    anim.isLooping = true;
    anim.speed  = speed;
    anim.pov = frost.Animator.AnimatorPointOfView.Free;
    anim.Animate(true);
    conGUI = frost.Animator.AnimatorControls();
    conGUI.anim = anim;
end