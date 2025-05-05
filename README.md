# Learning to Adapt through Bio-Inspired Gait Strategies for Versatile Quadruped Locomotion

We present a bio-inspired quadruped locomotion framework that exhibits exemplary adaptability, capable of zero-shot deployment in complex environments and stability recovery on unstable terrain without the use of extra-perceptive sensors. Through its development we also shed light on the intricacies of animal locomotion strategies, in turn supporting the notion that findings within biomechanics and robotics research can mutually drive progress in both fields.

This repository acts to supplement our paper and as such here you will find sample demos of our framework deployed on a range of terrains while realising a diverse range of gaits for preserving efficiency and stability. We also provide all data used to generate the figures within the paper.

A preprint of the paper is available here: https://arxiv.org/abs/2412.09440

-------------------

## Installation Guide

### Prerequisites

- Supported OS: Ubuntu(20.04 has been tested)
- Python version: 3.9


### Dependencies
| Software/Package | Version |
|:--:|:--:|
| [RaiSim](https://raisim.com/) | \>=1.1.6 |
| [PyTorch](https://pytorch.org/) | Platform Specific (tested with 1.10.2+cu113) |
| [Colcon](https://colcon.readthedocs.io/en/released/) | - |
| [Pybind11](https://pybind11.readthedocs.io/en/stable/index.html) | \>=2.3.4 |
| [SciPy](https://scipy.org/) | \>=1.10.1 |
| [NumPy](https://numpy.org/) | \>=1.24.2 |
| [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) | - |

### Workspace Setup and Install
First, create a workspace and `src` folder, replacing `<workspace_folder>` with a sensible name such as learningtoadapt_ws:
```
mkdir -p <workspace_folder>/src
```
Next this repository should be cloned within `src` and then can be built using Colcon at the `<workspace_folder>` level:
```
cd <workspace_folder>/src
git clone https://github.com/ihcr/bio_gait.git
cd ..
colcon build
```
Once the code has been compiled and locally installed, you can source .bash file in `install/setup.bash`
```
. install/setup.bash
```

## Running Demos
To run a demo, in a new terminal (not the one where you sourced the .bash file) terminal launch the Raisim visualiser of your choise (raisimUnity is recommended) and in the terminal where the .bash file was sourced run the following command, replacing `<demo_name>` with the demo you want to run:
```
python3 src/learning_to_adapt/scripts/run_demo.py /configs/loco_bio_gs_unified.yaml <demo_name>
```
The demos available are as follows:
- `sprint`
    - Sprinting in a straight line on flat terrain. Demonstrates how the gait selection policy selects the optimal gait for efficiency.
- `sprint_terr`
    - Sprinting in a straight line on rough terrain. Demonstrates how the gait selection policy selects the optimal gait for efficiency and stability and the locomotion policy can adapt to this previously unobserved terrain.
- `stresstest`
    - Realising a complex and challenging base velocity command on flat terrain. Demonstrates how the proficiency of the framework is upheld during highly demanding tasks.
- `stresstest_terr`
    - The same base velocity command in `stresstest` is used but with the addition of rough terrain to demonstrate how locomotion and gait selection policies adapt to this change in terrain to uphold proficiency.
- `planks`
    - Aims to mimic the loose timber hardware experiment seen in the paper to demonstrate how auxiliary gaits (pronk, bound, hop and limp) are used in conjunction with the nominal gaits (trot and run) to recover from critically unstable states.
- `allgaits`
    - All gaits are manually cycled through to demonstrate the diverse set of gaits the locomotion policy is capable of deploying.
- `allgaits_terr`
    - The same as `allgaits` but with the addition of rough terrain.
    
## Data
Within this repository we have also provided all data used to generate the figures within the paper. This data can be found within the `data` folder and is organised with the same labels as the paper.

## Cite
To reference this work, please use the following citation:
```
@misc{humphreys2024learningadaptbioinspiredgait,
      title={Learning to Adapt through Bio-Inspired Gait Strategies for Versatile Quadruped Locomotion}, 
      author={Joseph Humphreys and Chengxu Zhou},
      year={2024},
      eprint={2412.09440},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2412.09440}, 
}
```
