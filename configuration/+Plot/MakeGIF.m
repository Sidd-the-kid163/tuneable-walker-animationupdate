function MakeGIF(conGUI)
[q_all,t_all]=qtgenerator();
delay_time = 1/15;   % 15 FPS
repeat_times = 5;

% --- Build repeated trajectory ---
t = [];
tlast = 0;

q = [];
xoffset = 0;

for i = 1:repeat_times
    % Append time
    t_step = t_all + tlast;
    t = [t, t_step];
    tlast = t(end);

    % Append q but shift base_x (index 1)
    q_step = q_all;
    q_step(1,:) = q_step(1,:) + xoffset;
    q = [q, q_step];

    % Compute next offset
    xoffset = q(1,end) - q(1,1);
end

% --- Frame selection to match GIF timing ---
des_t = 0:delay_time:t(end);
diff_t = abs(repmat(t, length(des_t), 1) - des_t');
[~, des_inds] = min(diff_t, [], 2);

% --- Generate GIF ---
gif(fullfile("./gifscontrol/", 'sim.gif'), 'DelayTime', delay_time);

for i = des_inds'
    conGUI.anim.Draw(t(i), q(:,i))
    gif
end
end