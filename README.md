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

Install the python ml-agents package (this will also install TensorFlow and ml-agents-envs) with
```bash
python -m pip install mlagents==0.15.1 
```
in your favorite virtual environment.

Clone this repository or dowload the zip file and unzip it.

To download the required Unity environments, run 
```bash
python downloadEnv.py
```
at the root of the cloned github repo. It will create a directory called env.

### UNN Training : 

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
mlagents-learn config/config_gutter.yaml --run-id=BAMDelayAware --load  --env=env/ARG_ROBOT --width=756 --height=756 --time-scale 10  --env-args --delay ARG_DELAY --aware 0 
```

To test the delay unaware UNN module after training on the other simulated robots run 

```bash
mlagents-learn config/config_gutter.yaml --run-id=BAMDelayUnaware --load  --env=env/ARG_ROBOT --width=756 --height=756 --time-scale 10  --env-args --delay ARG_DELAY --aware 1
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
