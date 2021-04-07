# DelayAwareUNN


This repo is accompanying our paper "Delay Aware Universal Notice Network : real world multi-robot transfer".

<p align="center">
  <img src="/Ressources/transfert.png" width=70% />
</p>
<p align="center">
  
</p>



## Unity Experiments

### Requirements :
	- Python 3.7
	- ml-agents 0.15.1 
	- ml-agents-envs 0.15.0
	- TensorFlow 1.15.2 or higer

Install the python ml-agents package (this will also install TensorFlow and ml-agents-envs) and gdown with
```bash
python -m pip install mlagents==0.15.1 
```

```bash
pip install gdown 
```

in your favorite virtual environment.

Clone this repository or dowload the zip file and unzip it.

All the bash commands below need to be lauch from the root of the Unity directory
```bash
cd Unity/
```

To download the required Unity environments, run 
```bash
python downloadEnv.py
```
at the root of the cloned github repo. It will create a directory called env. (If you are on Linux, you may need to add the execution right for the environments).

```bash
mkdir log_output
```
to create the directory where the log_output will be stored. It will not run without this directory.

### UNN Training : 

For windows user, you must add /gutter_task to all env argument for the below commands, e.g
```bash
--env=env/BAM_training/gutter_task
```

To train the delay aware UNN module on the BAM robot run
```bash
mlagents-learn config/config_gutter.yaml --run-id=BAMDelayAware --train  --env=env/BAM_training --width=756 --height=756 --time-scale 10  --env-args --aware 0
```

To train the delay unaware UNN module on the BAM robot run
```bash
mlagents-learn config/config_gutter.yaml --run-id=BAMDelayUnaware --train  --env=env/BAM_training --width=756 --height=756 --time-scale 10  --env-args --aware 1
```

### UNN Transfer : 

To test the delay aware UNN module after training on the other simulated robots run 
```bash
mlagents-learn config/config_gutter.yaml --run-id=BAMDelayAware --load  --env=env/ARG_ROBOT --width=756 --height=756 --time-scale 1  --env-args --delay ARG_DELAY --aware 0 
```

To test the delay unaware UNN module after training on the other simulated robots run 

```bash
mlagents-learn config/config_gutter.yaml --run-id=BAMDelayUnaware --load  --env=env/ARG_ROBOT --width=756 --height=756 --time-scale 1  --env-args --delay ARG_DELAY --aware 1
```

with 	ARG_ROBOT = BraccioUNN_transfer for the braccio robot or
		ARG_ROBOT = 2DoFUNN_transfer for the 2 DoF robot

where   ARG_DELAY in [0.1 , 1] is the delay of the system
		
### Vanilla Training : 

To train the delay aware Vanilla agent on the robot run

```bash
mlagents-learn config/config_gutter.yaml --run-id=(ARG_ROBOT)DelayAware --train  --env=env/ARG_ROBOT --width=756 --height=756 --time-scale 3  --env-args --aware 0
```

To train the delay unaware Vanilla agent on the robot run

```bash
mlagents-learn config/config_gutter.yaml --run-id=(ARG_ROBOT)DelayUnaware --train  --env=env/ARG_ROBOT --width=756 --height=756 --time-scale 3  --env-args --aware 1
```

with 	ARG_ROBOT = VanillaBraccio for the braccio robot or
		ARG_ROBOT = Vanilla2DoF for the 2 DoF robot

<p align="center">
  <img src="/Ressources/BAM.gif" width=70% />
</p>
<p align="center">
UNN module on the BAM robot
</p>

<p align="center">
  <img src="/Ressources/Braccio.gif" width=70% />
</p>
<p align="center">
UNN module on the Braccio robot
</p>

<p align="center">
  <img src="/Ressources/2DoF.gif" width=70% />
</p>
<p align="center">
UNN module on the 2DoF robot
</p>

## ROS Experiments

### Requirements : 
	- Python 2.7
	- ROS Kinetic
	- TensorFlow 1.15.2 or higer
	- OpenCv


Once the models are fully trained on the simulated environments, the tensorflow model will be located inside model/run-id/My Behavior/frozen_graph_def.pb

To launch the ROS master run 
```bash
roscore
```
All the bash commands below need to be lauch from the root of the ROS directory

To try the models on the real robots, first launch the camera.py script which will track the required poses a
```bash
python camera.py
```
Pls note that the script expect the ball to be green and a yellow circle at one of the end of the gutter to get the ball position and velocity.

To run the UNN based models :
```bash
python unn.py -p ARG_PATH -m ARG_METHOD -r ARG_ROBOT -t ARG_TEST
```

or the Vanilla models run : 
```bash
python vanilla.py -p ARG_PATH -m ARG_METHOD -r ARG_ROBOT -t ARG_TEST
```

where 	ARG_PATH is the path to the tensorflow model
		ARG_METHOD is an integer indicating wether the model is delay aware (0) or not (1)
		ARG_ROBOT is an integer specifying the type of robot to use : braccio(0) or 2DoF(1)
		ARG_TEST is an integer specifying the type of test to run : short test (0) or long test (1). A short test is composed of 3 desired ball position whereas a long test is composed of 50.

<p align="center">
  <img src="/Ressources/Braccio_P.gif" width=70% />
</p>
<p align="center">
UNN module on the Braccio robot
</p>

<p align="center">
  <img src="/Ressources/2DoF_P.gif" width=70% />
</p>
<p align="center">
UNN module on the 2DoF robot
</p>
