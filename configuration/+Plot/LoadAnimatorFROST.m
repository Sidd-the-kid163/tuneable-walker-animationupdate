function [conGUI] = LoadAnimatorFROST(behavior)
[q,t]=qtgenerator();
robot_disp = Plot.LoadDisplay(behavior);
anim = frost.Animator.AbstractAnimator(robot_disp, t, q);
anim.isLooping = false;
anim.speed = 1;
% anim.isPlaying = 0;
anim.pov = frost.Animator.AnimatorPointOfView.Free;
% anim.updateWorldPosition = false;
anim.Animate(true);
conGUI = frost.Animator.AnimatorControls();
conGUI.anim = anim;
end