
# SETTING UP THE ENVIRONNEMENT 

***IMPORTANT: THIS DOCUMENTATION IS A WORK IN PROGRESS. IF YOU SEE ANY ERRORS, UNCLEAR STEPS OR OUTDATED INFO, PLEASE NOTICE US OR UPDATE IT***

Before you start setting up, it's important that your **UBUNTU 20.04** set up is done.
 
First of all, it is important to note that we do **virtual AND real-life racing**. Therefore, distinct setup-up are needed for each and you might want to only set-up the environment that you're going to work with.
They are **4 main repositories** being used in the project. One is for virtual racing : *Autodrive_roboracer_ws*. The three other are used for real-life racing: *sim_ws*, *ARCUS* and *VESC*.

# 1. Virtual racing
Our team is not ready yet to compete in virtual roboracer competitions. The setup is still a work in progress and documentation will come soon.
# 2. Real-life racing
For real-life racing,they are 3 important repository that you will find in the **Robotiques_udes github**. Don't worry, the documentation will walk you through the setup step by step.
## Overview 
The workflow to code the car for real-life racing revolves around 3 important aspects:
- **Development**
- **Simulation**
- **Uploading the code in the car**  

Our team decided to use docker to make life easier for everyone. A docker is a containerized environment on your host machine that has it's own set of dependencies.
## 2.2 Cloning the ARCUS repo
The ARCUS repo, that you can find in the **Robotiques_udes github**, is the team's repo with all of our ROS2 packages. This is the core of our codebase, and chances are this is where you'll be coding the most.
- Make sure you're in your home directory:
```bash
cd ~
```
- Clone the **ARCUS** repo:
```bash 
git clone https://github.com/Gabduf/ARCUS_ros_pkg_example.git
```
**!!! Les repos sont pas encore split comme on veut dans le github de robotiqueUdes donc en attendant, je propose de juste cloner un exemple de ROS2 package pour vite voir si ça marche. Mais keep in mind que à la place de ça, ça va être le ARCUS folder qui va contenir toutes nos ROS2 package... 
<u>
Pour que ça marche avec les dockers plus tard, renommer le folder ARCUS après l'avoir clôner !!!**
</u>

## 2.3 Setting up the sim_ws
Now that you have our ROS2 codebase on your host machine, you'll need the simulation workspace, allowing you to quickly test you're code in the f1tenth_gym, a custom tools built for f1tenth (now roboracer), on top of RViz.
- Make sure you're in your home directory:
```bash
cd ~
```
- Clone the **sim_ws** repo:
```bash 
git clone https://github.com/Gabduf/sim_ws.git
```
**!!! Encore une fois, là ça pointe à mon github en attendant, mais dans le futur ça va être dans le github de RobotiqueUdes.**

- Notice what's in sim_ws:
```
sim_ws/
├── docker-compose.yml
├── Dockerfile
└── src/
    ├── f1tenth_gym/
    ├── f1tenth_gym_ros/
    └── scripts/
```
1. The *Dockerfile* and the *docker-compose.yml* are the two files that will create the docker environnement. 
2. Their is also a src/ folder, whith *f1tenth_gym* and *f1tenth_gym_ros*. Think of the *f1tenth_gym* as the "simulation software". However, the f1tenth_gym_ros is essential to "bridge" our ROS2 package (in ARCUS) to the gym (allow "communication" between the simulation gym and our ROS2 node).
3. The scripts/ folder holds a couple of useful scripts, like the entrypoint script automating a couple of things for you everytime you build the *arcus* docker

## 2.3 Modifying your .bashrc
The *.bashrc* file is a **shell configuration file**. Everytime you open a terminal, a bash session starts, executing the *.bashrc*. You'll need to add a couple of macros and functions in your .bahsrc to work efficiently.

- Open the .bashrc with the gedit editor (simplest editor)
```bash
gedit ~/.bashrc
```
- At the top of the file, copy:
```
# Additions
alias arcus-build='xhost +local:docker && cd ~/sim_ws && docker-compose build'
alias arcus-up='cd ~/sim_ws && docker-compose up -d'
alias arcus-bash='docker exec -it arcus bash'
alias arcus-nuke='docker system prune -a --volumes'
alias arcus-space='docker system df'
alias arcus-list='docker ps'
export RCUTILS_COLORIZED_OUTPUT=1

b() {
    CONTAINER_NAME="arcus"
    docker exec -it "$CONTAINER_NAME" bash -c '
        source /opt/ros/humble/setup.bash
        cd /sim_ws || exit
        colcon build '"$*"'
        source install/setup.bash
    '
}
```
- Here's an important overview of what each macros and function mean:
1. *arcus-build* : this command will build the *arcus* docker container.
2. *arcus-up* : this command starts the *arcus* docker.
3. *arcus-bash* : this command allows you to start a bash session in the docker container. In other words, the docker container is isolated from your host machine. Therefore, you can't access or do anything in the container from your host terminal... You need to open the docker's terminal, with arcus-bahs.
4. *arcus-space* : this will give you a summary of the space being taken on your host machine by the docker images.
5. *arcus-nuke* : this command destroys every docker image. It can be pretty useful if you have plenty of docker images on your host machine taking a lot of space (check using arcus-space)
6. *arcus-list* : this command will give you the list of the dockers running on your host machine.
7. b : this function handles the building of all the ROS2 packages using *colcon build*. **IMPORTANT: You can run it from anywhere on your HOST terminal. Meaning you can't use the 'b' command from a docker bash session. The function handles it by going in the docker itself.**

## 2.3 Building and running the docker
 First, you'll need to install docker and related packages: 
 - Installation: https://docs.docker.com/engine/install/ubuntu/
 Docker also needs root-level access.
- Add docker group:
```bash
sudo groupadd docker
```
- Join group:
```bash
sudo usermod -aG docker $USER
```
- Reboot your computer:
```bash
reboot
```
- Ensure that it worked:
```bash
groups
```
You should see docker in the groups !
- Give permission to the entrypoint.sh script:
```bash
chmod +x ~/sim_ws/src/scripts/entrypoint.sh
```
- Build the docker container:
```bash
arcus-build
```
(it may take a while, especially the first time, don't worry)
- Start the docker container:
```bash
arcus-up
```
- Check to make sure that the docker is running:
```bash
arcus-list
```

You should see a docker named *arcus* in the list !

**IMPORTANT : You'll need to get familiar with the folder structure in the docker. To do that, you can enter a bash session in the docker with `arcus-bash` , use `ls` the see the files/folders and `cd` to navigate.
This is the file structure IN the docker :
```
sim_ws/
└── src/
    ├── ARCUS/
    │   └── ros_kg/
    ├── f1tenth_gym/
    └── f1tenth_gym_ros/
```
Notice how it is another structure then on you're host machine. This is because we mount sim_ws/ and the ARCUS/ this way. Mounting allows the changes that you do IN the docker to immediately appear on your host machine (because yes, you will be coding in the container, and not on your host).

## 2.4 Testing the simulator with ROS2 

Now that the docker container is running with all the needed dependencies, we can test the simulation. 
- Build the ROS2 packages:
```bash
b
```
- Open a bash session in the docker:

```bash
arcus-bash
```
- Launch the simulation:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
- Launch the *gap_follower* package:
```bash
ros2 run gap_follow reactive_node
```
You should now see the car in the simulation moving around the track, using the *follow the gap* algorithm !

## 2.4 Developping using VSCode inside the docker
TODO