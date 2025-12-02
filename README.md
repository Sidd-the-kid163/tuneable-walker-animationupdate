# tunable-walker-trajopt
This repository contains the pre-generated gaits for the 5-link adjustable walker, as well as the code to generate new gaits.

## Control parameter files (pre-generated gaits)
Each of the 25 gaits are stored in the folder 'All_Gaits'. In particular, the '.yaml' file stores the bezier coefficients in the field 'aposition'. These parameters are of size nx8 with n=6. These 6 joints are associated with the joints:
1. Stance Leg Shin Extension
2. Stance Leg Knee Joint
3. Stance Leg Thigh Extension
4. Swing Leg Hip
5. Swing Leg Knee
6. Swing Leg Shin Extension

To load a stored gait from the yaml, use:
```matlab
yaml = yaml_read_file('All_Gaits/gait_1_1/params_gait_1_1.yaml',true,false)
gait = yaml.domain{1}
control_points = reshape(gait.aposition,6,8)
```

Then, for example, if you wanted to plot the trajectory of the stance knee, you could run the following
```matlab
sl_knee_cp = control_points(2,:)
phase_vals = linspace(0,1);
knee_vals = bezier(sl_knee_cp,phase_vals);
plot(phase_vals, knee_vals);
```

## Gait Generation Code
For a tutorial on FROST, I recommend following the frost example repository (https://github.com/dynamicmobility/rabbit-opt-example). This repository is a variation in which the parameters are more easily adjustable for easier personalization.

### Installation
1. This example requires the following dependencies:
	- MATLAB 2022b or later (Can be installed from GT software portal) with the following add-ons:
        - Curve Fitting Toolbox
        - Optimization Toolbox
        - Symbolic Math Toolbox
        - Robotics System Toolbox
        - Control System Toolbox
	- Mathematica 12X (Requires special permission from OIT to install)
2. FROST is a toolbox which is set up as a submodule for this repo. Make sure you have cloned the repo including submodules, then set up
FROST following the instructions provided [here](https://ayonga.github.io/frost-dev/pages/installation.html)
- Note that you may need to add the following line to your .bashrc in order for FROST to successfully locate certain linked libraries.
    ```
    LD_LIBRARY_PATH=/usr/local/Wolfram/Mathematica/12.0/SystemFiles/Links/WSTP/DeveloperKit/Linux-x86-64/CompilerAdditions:$LD_LIBRARY_PATH
    ```
3. Install the SnakeYAML Library in MATLAB by running `setupMatlabYaml.m`
   located in the folder `tools/matlabYaml`. 
4. When you run the `main_opt.m` script, make sure you ran MATLAB from your
   terminal. This ensures that the environment variables that were set in step 3
   were correctly loaded. Otherwise, expect an error similar to the following to
   show up when you run `frost_addpath`:
   ```
   Invalid MEX-file <your_path>: libWSTP64i4.so: cannot open shared object file: No such file or directory
   ```
5. The first time you run the main_opt script, you will need to compile the .mex files required to run the gait generation problem. To set the compile flags to true, change lines 39 and 48 of main_opt to:
    ```
    compileMex = [1,1];
    do_compile = [1,1,1];
    ```
    After compiling the mex files, you can set these lines to false (0). Note that you will need to recompile the .mex files if you change the gait generation problem setup.
6. Run the entire `main_opt.m` script to generate and simulate a gait

### Run TrajOpt
To run the main trajectory optimization code, run the `MAIN_SCRIPT.M` script. This generates the GIFs for all 25 pre-generated gaits.