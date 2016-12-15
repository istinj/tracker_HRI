# HRI-LAS Project Repository

### Goal
1. Il robot ha delle coordinate globali nel mondo (*truth*)
2. Bag proveniente dalla camera
3. **Detection** di ostacoli generici tramite *laserscan*
4. **Tracking** tra *robot pose* ed *estimated human pose* (???)
5. **Analisys** per riconoscere persone tramite *rgb + depth*
6. Pubblicazione di condizione -> **PNP**


### TODO:
- Fare in modo da filtrare le depth in base ai valori ottenuti dal laserscan.
- Fare un nuovo launch in modo da far partire solo i pacchetti necessari (?????).
- Modificare il plan (non va bene).
- Registrare le azioni nel *action_server* - al momento solo dei placeholder.

###### Ricorda
*LASERSCAN->RANGE* = **METRI** // *DEPTH_IMAGES* = **MILLIMETRI**


#### UNDERSTAND AND MODIFY ROS PACKAGE FOR HRI-PNP
hri_pnp package has the following structure:

- **src**: contains the action server (HRIPNPAS.cpp) and implementation of actions
  (Actions.cpp) and conditions (Conditions.cpp)
- **plans**: contains the PNPs
- **launch**: contains a launch file for pnp_ros and tcp_interface nodes
- **scripts**: contains a script to run the simple demo

To *extend* this project, consider the following steps:

1. Create new PNPs in the plans folder
2. Implement new actions and condition in the ```src/*.cpp``` files
3. Extend launch file if new nodes need to be added.
4. Create new script to launch the new demo - *bash*
