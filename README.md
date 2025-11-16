# Origins-W1

This is the first release of a manual control program that allows the user to control the f1tenth chassis using a ps4 controller. The end goal of this project is to make an autonomous ground vehicle from this platform. 


## Hardware 

The necessary hardware for this project includes: 
- PS4 Controller
- F1tenth Chassis
- Jetson Orin Nano
- VESE MKVI made by Trampa


![IMG_3132](https://github.com/user-attachments/assets/e2849557-18af-4cf9-9cf7-0f933d829d3b)

The Base Folder contains all of the necessary files to control the rover from a laptop or base station. The Rover folder contains all of the files necessary to run locally on the jetson. 

To run these files, on the base station run base.py, and on the jetson run rover.py. It is recommmended to select bypass PID as the controller is still in early stages. In order to activate the controller simply select enable controller. 
Be sure to connect the controller before running base.py. 


## Video of the Rover Moving 

