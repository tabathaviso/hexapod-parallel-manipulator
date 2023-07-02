# hexapod-parallel-manipulator

Consider a Stewart Platform parallel manipulator hexapod
The diameters of the top and base platform are 250mm and 650mm
Nominal and real kinematic parameters are derived from Tables 1 and 2 of
"A total solution to kinematic calibration of hexapod machine tools with a minimum number of measurement configurations and superior accuracies", MJ Nategh, MM Agheli (2009)

When the desired pose of the top platform is given, inverse kinematics (IK) can be used to find the leg lengths.
Applying these leg lengths will theoretically take the robot to the expected configuration. 
The error propogation in the workspace (as a result of errors in the kinematic parameters) is derived. 


