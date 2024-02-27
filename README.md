# multi-agent-systems-formation-control

## Owners: 

Giulia Ciabatti, 
Leonardo Brizi, 
Eleonora Chiarantano


Implementation of three distributed model predictive control (MPC) methods for formation control of unicycles:

* **Paper A**: M. A. Kamel and Y. Zhang, "Linear model predictive control via feedback linearization for formation control of multiple wheeled mobile robots", 2015 IEEE International Conference on Information and Automation, Lijiang, China, 2015, pp. 1283-1288;
* **Paper A (extension)**: M. A. Kamel and Y. Zhang, "Decentralized Leader-Follower Formation Control with Obstacle Avoidance of Multiple Unicycle Mobile Robots", 2015 IEEE 28th Canadian Conference on Electrical and Computer Engineering (CCECE), 2015, pp. 406-411;
* **Paper B**: H. Fukushima, K. Kon and F. Matsuno, "Distributed Model Predictive Control for Multi-Vehicle Formation with Collision Avoidance Constraints", Proceedings of the 44th IEEE Conference on Decision and Control, Seville, Spain, 2005, pp. 5480-5485;

## Simulation Requirements

In order to use the CoppeliaSim (aka VRep) scenes, [CoppeliaSim robot simulator](https://www.coppeliarobotics.com/downloads) must be installed. Add the libraries necessary for the communication with CoppeliaSim, copying them from the CoppeliaSim installation folder:
* `remApi.m`
* `remoteApi.so`
* `remoteApiProto.m`

in a subfolder `vrep_libraries` in the current workspace.
