# Cartesian Impedance Controller
This repository contains a Cartesian Impedance Controller, based on Elastic Potentials. More Details can be found here:

    ```
    @phdthesis{lachner2022geometric,
    title={A geometric approach to robotic manipulation in physical human-robot interaction},
    author={Lachner, Johannes},
    school={University of Twente},
    year={2022}
    }
    ```

# Setting up the software 
This software can be used in combination with Exp[licit]-MATLAB, e.g., by adding this repository as a submodule. The documentation of Exp[licit] can be found [here](https://explicit-robotics.github.io/).

Once this repository has been added as a submudule **on the same folder level** as Explicit-MATLAB, adapt the following code in **Explicit-Matlab/setup.m**
```
    % Include all the subdirectories
    func_addSubfolders( 'animation', 'helpers_geometry', 'robots',...
                   'examples', 'interpolator', 'utils', 'graphics', '../CartImpedanceControl' );
```
Note: In this example, *CartImpedanceControl* is the name of your submodule. 

Now you can create an instance of `CartImpController` for the examples in Explicit-MATLAB (e.g., **Explicit-Matlab/examples/main_iiwa14.m**)
```
    impCtrl_p = CartImpController( 'translational' );
    impCtrl_p.setKinematics( H );
    impCtrl_p.setZFT( p_des );
    impCtrl_p.setStiffness( K_p );
    F_p = impCtrl_p.getWrench( );
    % ...
```

# Authors
This software is under development by [Johannes Lachner](https://jlachner.github.io/).