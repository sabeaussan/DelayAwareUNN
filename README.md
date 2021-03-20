# DelayAwareUNN


This repo is accompanying our paper "Delay Aware Universal Notice Network : real world multi-robot transfer".

<p align="center">
  <img src="/Ressources/transfer.png" width=70% />
</p>
<p align="center">
  
</p>

## Requirements :
	- Python 3.7
	- ml-agents 0.15.1 
	- ml-agents-envs 0.15.0
	- TensorFlow 1.15.2



## Unity Experiments

### UNN Training : 

To train the delay aware UNN module on the BAM robot run
```bash
mlagents-learn config/config_gutter.yaml --run-id=GutterBAMDelayAware --train  --env=env/MAC/BAM_training --width=756 --height=756 --time-scale 10  --env-args --aware 0
```

To train the delay unaware UNN module on the BAM robot run
```bash
mlagents-learn config/config_gutter.yaml --run-id=GutterBAMDelayUnaware --train  --env=env/MAC/BAM_training --width=756 --height=756 --time-scale 10  --env-args --aware 1
```

### UNN Transfer : 

To test the delay aware UNN module after training on the other simulated robots run 
```bash
mlagents-learn config/config_gutter.yaml --run-id=GutterBAMDelayAware --load  --env=env/MAC/ARG_ROBOT --width=756 --height=756 --time-scale 10  --env-args --delay ARG_DELAY --aware 0 
```

```bash
mlagents-learn config/config_gutter.yaml --run-id=GutterBAMDelayUnaware --load  --env=env/MAC/ARG_ROBOT --width=756 --height=756 --time-scale 10  --env-args --delay ARG_DELAY --aware 1
```

with 	ARG_ROBOT = BraccioUNN_transfer for the braccio robot or
		ARG_ROBOT = 2DoFUNN_transfer for the 2 DoF robot

where   ARG_DELAY in [0.1 , 1] is the delay of the system
		
### Vanilla Training : 

To train the delay aware Vanilla agent on the robot run

```bash
mlagents-learn config/config_gutter.yaml --run-id=(ARG_ROBOT)DelayAware --train  --env=env/MAC/ARG_ROBOT --width=756 --height=756 --time-scale 10  --env-args --aware 0
```

To train the delay unaware Vanilla agent on the robot run

```bash
mlagents-learn config/config_gutter.yaml --run-id=(ARG_ROBOT)DelayUnaware --train  --env=env/MAC/ARG_ROBOT --width=756 --height=756 --time-scale 10  --env-args --aware 1
```

with 	ARG_ROBOT = VanillaBraccio for the braccio robot or
		ARG_ROBOT = Vanilla2DoF for the 2 DoF robot
