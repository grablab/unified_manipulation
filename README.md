# Unified Arm-Hand Manipulation via Optimization-based Motion Synthesis

Repo for motion synthesis code and experiment results from analyzing unified manipulation on arm-hand combinations.

Paper was accepted to IEEE ICRA 2023! Corresponding email: `v.patel@yale.edu`.

```
@inproceedings{patel2023unified,
  title={An Analysis of Unified Manipulation with Robot Arms and Dexterous Hands via Optimization-based Motion Synthesis},
  author={Patel, Vatsal V and Rakita, Daniel and Dollar, Aaron M},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  organization={IEEE}
}
```

- [Aggregated Results](#aggregated-results-from-paper)
- [KUKA Arm Videos](#kuka-results)
- [UR10 Arm Videos](#ur10-results)
- [Task-based Videos](#task-based-results)
    - [Smooth Helical Task](#a-smooth-helical-task)
    - [Sharp Path Task](#b-sharp-path-task)
    - [Cup Pour Task](#c-cup-pour-task)
    - [I-C-R-A Collide Task](#d-icra-collide-task)
    - [Small Movt Task](#e-small-movt-task)
- [More Weight Versions Results](#more-objective-weight-versions)

## Aggregated Results from Paper

The aggregated results as shown in the paper for both the KUKA and UR10 arms on all 5 experiment tasks are as follows.

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table1.png)

Optimization weight versions tested:
- *Arm-Hand-Config α* prioritizes object pose accuracy
- *Arm-Hand-Config β* prioritizes joint and object motion smoothness

Additional weight versions that prioritize other motion attributes were also tested, and those results are [below](#more-objective-weight-versions)!

## KUKA Results

The results for the 7-DOF KUKA arm with the diffrernt hands on all 5 experiment tasks are as follows.

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table2.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/smooth-helical-kuka.gif" width="30%" height="30%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/sharp-path-kuka.gif" width="30%" height="30%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/cup-pour-kuka.gif" width="30%" height="30%"/>
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/icra-collide-kuka.gif" width="30%" height="30%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/small-movt-kuka.gif" width="30%" height="30%"/>
</p>
  
## UR10 Results

The results for the 6-DOF UR10 arm with the diffrernt hands on all 5 experiment tasks are as follows.

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table3.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/smooth-helical-ur10.gif" width="30%" height="30%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/sharp-path-ur10.gif" width="30%" height="30%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/cup-pour-ur10.gif" width="30%" height="30%"/>
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/icra-collide-ur10.gif" width="30%" height="30%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/small-movt-ur10.gif" width="30%" height="30%"/>
</p>

## Task-based Results

The results for each experiment task averaged across the 2 robot arms (KUKA and UR10) are shown here. The alpha weight versions are shown here for brevity.

### (a) __Smooth Helical__ Task

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table4.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/smooth-helical-kuka.gif" width="40%" height="40%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/smooth-helical-ur10.gif" width="40%" height="40%"/>
</p>

### (b) __Sharp Path__ Task

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table5.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/sharp-path-kuka.gif" width="40%" height="40%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/sharp-path-ur10.gif" width="40%" height="40%"/>
</p>

### (c) __Cup Pour__ Task

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table6.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/cup-pour-kuka.gif" width="40%" height="40%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/cup-pour-ur10.gif" width="40%" height="40%"/>
</p>

### (d) __ICRA Collide__ Task

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table7.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/icra-collide-kuka.gif" width="40%" height="40%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/icra-collide-ur10.gif" width="40%" height="40%"/>
</p>

### (e) __Small Movt__ Task

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table8.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/small-movt-kuka.gif" width="40%" height="40%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/small-movt-ur10.gif" width="40%" height="40%"/>
</p>

## More Objective Weight Versions

Results from other optimization weight versions tested are below:
- *Arm-Hand-Config γ* prioritizes avoiding singularities and joint limits
- *Arm-Hand-Config δ* prioritizes reducing joint torques
- *Arm-Hand-Config ε* prioritizes collision avoidance

![alt text](https://github.com/grablab/arm_hand_config/blob/main/png/table9.png)

<p align="center">
<img src="https://github.com/grablab/arm_hand_config/blob/main/png/kuka-sharp-path-gamma.gif" width="32%" height="32%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/kuka-sharp-path-delta.gif" width="32%" height="32%"/><img src="https://github.com/grablab/arm_hand_config/blob/main/png/kuka-sharp-path-epsilon.gif" width="32%" height="32%"/>
</p>
