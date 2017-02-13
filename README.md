# HRI-LAS Project Repository

## Goal
1. Bag proveniente dalla camera
2. **Detection** di ostacoli generici tramite *laser_analysis*
3. **Tracking** della posizione dell'ostacolo trovato da laser_analysis tramite *EKF*
4. **Analisys** basata su *depth_ROIs* (se una ROIs ha una certa dimensione -> è un umano)
5. Pubblicazione di condizione 'pDetected' -> **PNP**

## Build and usage
1. Setup a catkin workspace
2. Include in the srcs the packages: **thin_state_publisher**, **thin_msgs** , **thin_localizer** and **laser_analysis**
3. ```catkin_make```
4. Adjust the parameters in the bash script ```run_tracker.sh``` (e.g. the initial position of the robot in the map or the **bag location**).
5. launch ```run_tracker.sh``` from the project root directory.

I used to play the bag that can be found at the following [link](https://drive.google.com/open?id=0B1SfHhyddX5xeDNXUFJPZ1JqV1E). Therefore, the initial position given to *thin_localizer* in ```run_tracker.sh``` is tailored on that bag. Otherwise *thin_localizer* can be used without a initial configuration and it will perform localization, but lowering the performances of the human tracker. So if you want to use the package with another bag, make sure that a suitable initial configuration is given as parameter of *thin_localizer* in ```run_tracker.sh```.
