This is the repository for my [Bachelor Thesis](https://github.com/halpersim/parameter_identification/blob/main/Bachelor_Thesis_Simon_Halper.pdf) _Identification of the dynamic parameters of the KUKA LBR iiwa_ at the Automation and Control Institute, TU Wien. 

*Abstract*
The dynamic parameters of a robot are a set of parameters that allow the simulation
and control of a robot’s motion. Although such a set of parameters is crucial for working
with any robot, it is not publicly available for the KUKA LBR iiwa R820. Therefore, the
dynamic parameters of the KUKA LBR iiwa R820 are estimated in this work. This is
done on the basis of the methods proposed in [1] using a nonlinear optimization solver.
The results of the parameter identification are verified by simulating the KUKA LBR
iiwa R820 with the identified parameters and comparing the simulation results with the
robot’s measurements.


Chapter 3.1 of my thesis is implemented in the Maple file [/simulation/kukalbriiwa_model/maple/withoutLinAxes/model.mw](https://github.com/halpersim/parameter_identification/blob/main/simulation/kukalbriiwa_model/maple/withoutLinAxes/model.mw).   
Chapter 3.2 is implemented in folder *paramter_identification*.   
Chapter 3.3 is implemented in folder *trajectory_generation*.    
