# eboat_gz_1

# EBoat Virtual Ocean Environment
# Álvaro's Version


---

\\
Gazebo/ROS based virtual environment to reinforcement learning research on USVs.

First of all you should have ROS Noetic and Gazebo 11 install in your system. For installation instructions see http://wiki.ros.org/noetic.

Once you have ROS and Gazebo installed, to make the environment run you will need to do:

1 - Create a workspace for the project

> $ ```mkdir -p ~/eboat_ws/src```

2 - Change to the src directory

> $ ```cd ~/eboat_ws/src```

3 - Clone de repository

> $```git clone https://github.com/medialab-fboat/eboat_gz_1.git```

4 - Change to the workspace directory

> $ ```cd ~/eboat_ws```

5 - Compile the source

> $ ```catkin_make```

6 - Source de ROS package

> $ ```source ~/eboat_ws/devel/setup.bash```

6.1 - Alternatively, you can insert the command above in your .bashrc file

> $ ```echo "source ${HOME}/eboat_ws/devel/setup.bash" > ${HOME}/.bashrc```

7 - Launch the environmen
> $ ```roslaunch eboat_gazebo ocean.launch```


---

\\
To launch the GUI with boat controls, you sould do:

1 - Install miniconda from https://docs.conda.io/en/latest/miniconda.html

2 - Create an anaconda environment using the YML file in ~/eboat_ws/src/eboat_gz_1/eboat_control/scripts/

> $ ```conda env create --file ${HOME}/eboat_ws/src/eboat_gz_1/eboat_control/python/script/ctr_gui.yml```

3 - Make the script executable

> $ ```chmod +x ~/eboat_ws/src/eboat_gz_1/eboat_control/scripts/controls_v0.py```

4 - **Change the path on the python script to point your ctrgui anaconda environment**

5 - Launch eboat_gazebo (If you did not follow installation step 6.1, you might need to execute installation step 6 before starting the environment)

> $ ```roslaunch eboat_gazebo ocean.launch```

6 - Run the python script

> $ ```~/eboat_ws/src/eboat_gz_1/eboat_control/scripts/controls.py```

\\

**OBS:** Alternatively, if you do not wish to make the python code executable, you can replace steps 3 to 6 with the following steps.

3' - Activate the environment

> $ ```conda activate ctrgui```

4' - Launch eboat_gazebo

> $ ```roslaunch eboat_gazebo ocean.launch```

5' - Run the python script

> $ ```python ~/eboat_ws/src/eboat_gz_1/eboat_control/scripts/controls.py```



---

\\
To make Gym environment avaiable for RL developing, you must do:

1 - Install miniconda from https://docs.conda.io/en/latest/miniconda.html

2 - Create an anaconda environment using the YML file in ~/eboat_ws/src/eboat_gz_1/

> $ ```conda env create --file ${HOME}/eboat_ws/src/eboat_gz_1/gazgym.yml```

3 - Activate the environment

> $ ```conda activate gazgym```

4 - Clone the GazeboGym environment from GitHub in your home folder

> $ ```git clone https://github.com/erlerobot/gym-gazebo.git```

5 - Got to the GazeboGym home

> $ ```cd ~/gym-gazebo```

6 - Install the environment

> ```pip install -e .```

\\

This project contains some gym environments predifined. You can use them or create your own environments for your RL tasks. The environments are stored in ~/eboat_ws/src/eboat_gz_1/eboat_gym_gaz/envs/

\\
\* The original YML file will install tensorflow-gpu and pythorch-gpu. In case you do not have a compatible GPU available, remove tensorflow-gpu and pythorch-gpu from the YML file.
