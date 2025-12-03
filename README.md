# tunable-walker-animationupdate
This repository is a handover of the https://github.com/dynamicmobility/tunable-walker-trajopt and builds on the animation capabilities of the frost + 5-link walker optimization code in two ways: First, we try to use the control points from the YAML file to directly animate the walker without the need for optimization/logger structure. Secondly, we improve the visibility of the animation by providing coordinates of the joints in a legend adjacent to the animation. An explanation of the files created:

## MAIN SCRIPT.m
It compiles all the code for optimization, connection to frost and snake yaml. This iterates over gait movements that are influenced by parameters: step length and thigh length. For testing purposes, you can use to avoid optimizing every time besides from commenting out the for loops:
```save('loggertempsave.mat'),'logger')```

A few solutions to keep in mind when optimizing:
1. Make sure to install snakeyaml and add to path.
2. When using original tunable walker, add this to make sure the integrals compile before use:
   ```OPT.compile(nlp,[],[],export_path)```
3. Fix export path (add your own) in export.m
4. When using original tunable walker, remember to reset logger after each iteration to avoid carryover.

## frostanimator.m
This is analogous to the main script for using control points to animate the robot walker.

## qtgenerator.m
This file compiles the yaml file and converts it to control points for each of the 8 joints:
1. X_base [Taken to be 0 since control points do not track base. Same for Y_base and Base_theta]
2. Y_ base
3. Base_theta
4. Stance Hip [Taken to be 0]
5. Stance Thigh Extension
6. Stance Knee
7. Stance Shin Extension
8. Swing Hip
9. Swing Thigh Extension [Taken to be 0]
10. Swing Knee
11. Swing Shin Extension
Stance leg is taken to be the left leg and swing being the right leg.

## Other common issues for both animation and optimization:
1. Note that this repository and the one it is based to are similar to the rabbit example (https://github.com/dynamicmobility/rabbit-opt-example) and can be used for duplication in case of confusion.
2. Mathematica (Wolfram) does not include the latest WSTP files (only contains the ML files). Make sure to make a copy of it from the wolfram foler containing the WSTP files and place it in the Math Link folder.
3. Original Tunable-Walker repo does not contain the matlabYaml folder for actions related to YAML files.
4. FROST uses a legend BlockDiagonalMatrix which conflicts with that of Mathematica. So this name needs to be changed to something else in the original repo.
5. Configuration directories are inaccessible in the original repo and needs to be fixed.

## RunOpt.m
This is the file where optimization takes place.

##All_Gaits
Contain the gifs and cost underlined by each optimization creating a gif with a different step length and thigh length from 1-5 for each. The gifs folder contains the gifs associated with each of these paramaters. To see how the legend is added for showcase of coordinations, see gait_1_1, gait_1_2, and gait_1_2_3 in the gif folder.

### EXPORTVIRTUALCONSTRAINT.m
This file is used to transfer vc behavior system of the robot for use in LLM research. This creates vc.json

### loggertempsave.mat
This file stores the logger variable for use in future testing (Belongs to gait 1_1).

### simplearraycomptest.py
This file contains a simple MSE test to compare between two arrays containing 6 x 8 control points (handy when testing LLM model capability in Bezier parameterization). As is known, since there are 8 control points for each joint, we use the 7th order Bezier curve to represent the joint points.

### yamlmatlab
This folder (https://github.com/ewiger/yamlmatlab) is a handover from Yauhen Yakimovich that provides easier YAML actions. Preferred over the original YAML tooling, unless specifically needed.

### frost-dev
FROST: Fast Robot Optimization and Simulation Toolkit (http://ayonga.github.io/frost-dev)

### URDF
Contains a description of the robotic system, key to animation and optimization. Also used to explain to the LLM model easily what the robotic system is.
