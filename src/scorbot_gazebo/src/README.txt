Tests:
------

- join_test.py: a simple test to test the scorbot. Each angle is sent alone. Use Gazebo demo with "scorbot_join_test.launch".

-  joinTrajectoryCommand_test.py: a simple test to test the scorbot. Angles are sent in a trajectory structure all together. Use Gazebo demo with "scorbot_joinTrajectoryCommand_test.launch".

- scorbot_direct_kinematics.py: script to perform the direct kinematics. Input: 4 robot angles. Output: (x,y,z) coordinates. Resultas where compared with [1]. In fact, in [1] there was an error in 0 4 T matrix.

- scorbot_inverse_kinematics_01.py: inverse kinematic script based in [2]. Input: (x,y,z) coordinates. Output: 4 robot angles.

- scorbot_inverse_kinematics_02.py: inverse kinematic script based in [1]. Input: (x,y,z) coordinates. Output: 4 robot angles.



References
----------

[1] Kinematics based on techreport "Drawing using the Scorbot-ER VII Manipulator Arm".  
Authors: Luke Cole, Adam Ferenc Nagy-Sochacki and Jonathan Symonds (2007)
https://www.lukecole.name/doc/reports/drawing_using_the_scorbot_manipulator_arm.pdf

[2] On the kinematics of the scorbot ER-VII robot. Authors: Laurenţiu Predescu,  Ion Stroe
https://www.scientificbulletin.upb.ro/rev_docs_arhiva/fulld8b_273573.pdf 
