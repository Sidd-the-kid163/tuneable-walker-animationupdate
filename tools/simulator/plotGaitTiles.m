function plotGaitTiles(anim,logger,folder)
    
t_log = logger(1).flow.t;

% Plot 8 gait tiles
times = linspace(t_log(1),t_log(end),8);

for i = 1:8
anim.Animate(true); %freeze animation
anim.Draw(times(i),anim.GetData(times(i)));
anim.updateWorldPosition = true;
grid(gca,'off')
figname =  fullfile(folder,sprintf('GaitTile%i.png',i));
saveas(gcf,figname)
end