# 🌊 CoUGARs Belief-Space MPPI

[![ROS 2 Build & Test](https://github.com/cougars-auv/coug_belief_mppi/actions/workflows/ros2_build_test.yaml/badge.svg)](https://github.com/cougars-auv/coug_belief_mppi/actions/workflows/ros2_build_test.yaml)
[![Docker Build](https://github.com/cougars-auv/coug_belief_mppi/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/cougars-auv/coug_belief_mppi/actions/workflows/docker_build.yaml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/cougars-auv/coug_belief_mppi/main.svg)](https://results.pre-commit.ci/latest/github/cougars-auv/coug_belief_mppi/main)
[![codecov](https://codecov.io/gh/cougars-auv/coug_belief_mppi/graph/badge.svg?token=92GLUNI35L)](https://codecov.io/gh/cougars-auv/coug_belief_mppi)

<p align="left">
  <img src=".github/assets/rviz.gif" width="600" alt="RViz demonstration of belief-space MPPI control">
</p>

## Get Started

> **Prerequisites:** 64-bit Linux, 5+ GB of free disk space.

- Install [Docker](https://www.docker.com/get-started/).

- Clone the `coug_belief_mppi` repository.

  ```bash
  git clone https://github.com/cougars-auv/coug_belief_mppi.git
  ```

- Enable GUI forwarding, build the Docker image, and launch the demo container.

  ```bash
  xhost +local:docker
  cd coug_belief_mppi/demo && docker compose up
  ```

## Results

### Challenges

There were several challenging aspects of this project, including Nav2 integration debugging and deriving the Jacobeans for the EKF predict and update steps in the BeliefStateCritic.

The two most challenging aspects, however, were (1) getting the code to run fast enough for real-time control, and (2) effectively tuning the trace heuristic. Even using OpenMP for CPU parallelism and performing extensive pre-computation, I was only able to run 250 rollouts of 15 steps of 0.1 seconds executing at 10 Hz on my lab computer with all 32 CPUs maxed out at 100%. Additionally, the full unnormalized covariance trace was dominated by the growing positional uncertainty inherent to dead reckoning and skewed by mixed units. To maximize the heuristic's impact over just a 1.5-second horizon, I chose to score the trajectory using the normalized IMU bias covariances instead.

### Experiments & Validation

To isolate and validate the performance of the BeliefStateCritic, I ran 6 missions – 3 with only the BeliefStateCritic enabled, and 3 with it disabled – and monitored the normalized IMU bias covariance trace over time. The BeliefStateCritic effectively identified uncertainty-reducing maneuvers as shown in the plot below.

<br>

<p align="center">
  <img src=".github/assets/norm_trace.png" width="400" alt="Normalized IMU bias covariance trace with versus without the BeliefStateCritic">
</p>

<p align="center">
  <em>Fig. 1. Normalized IMU bias covariance trace with versus without the BeliefStateCritic.</em>
</p>

<br>

The demo demonstrates the BeliefStateCritic encouraging excitation-rich maneuvers as the AUV navigates to 3 waypoints using the MPPI controller.

### AI Use Disclosure

Gemini 3.1 Pro was used as a collaborative partner for initial brainstorming, and Claude Opus 4.6 was used for code simplification and speed suggestions.

## Elevator Pitch

### Problem

Effective underwater navigation requires balancing both progressing toward a goal and maintaining an accurate state estimate. For AUVs relying extensively on internal sensors (IMU, DVL, etc), excitation – changes in acceleration and velocity – can often improve state estimate accuracy. This conflicts with the goal-directed approach of many AUV path planners and controllers.

Finding the optimal trajectory that balances goal achievement with state estimation accuracy is a POMDP. However, through extending our state vector to include the covariance matrix of our state estimate, we can transform the problem into an observable belief-state MDP. This is a common strategy in Active SLAM, but results in a high degree of dimensionality. Calculating an exact closed-loop solution across all those continuous dimensions raises some serious computational challenges, especially in real-time on limited AUV hardware.

### Proposed Solution Approach

To solve this continuous-time MDP in real-time, I plan to implement an open-loop Model Predictive Path Integral (MPPI) controller, which is the continuous-time version of the Multiforecast Model Predictive Control (MMPC) approach described in section 9.9.3 of the textbook. During each planning cycle, the MPPI algorithm will sample a large number of stochastic trajectory rollouts, evaluate them against a heuristic that penalizes both deviation from the goal and the trace of the covariance matrix, and execute the first step of the averaged optimal control sequence.

To simplify the scope of the project, I will make use of the HoloOcean simulator and Nav2’s CPU-optimized “nav2_mppi_controller” class available in ROS 2. Specifically, I plan to:

1. Integrate Nav2’s “nav2_mppi_controller” with the HoloOcean simulator
2. Implement a new AUV dynamics plugin (based on a simplified Fossen model) to accurately model physical state propagation during rollouts
3. Implement a new heuristic plugin to simulate uncertainty propagation and penalize trajectories based on the trace of the resulting covariance matrix, forcing the the controller to favor excitation-rich action sequences
4. Benchmark the resulting algorithm’s uncertainty growth over a set waypoint sequence against a baseline MPPI controller

## Contributing

We **strongly recommend** using the [`cougars-dev`](https://github.com/cougars-auv/cougars-dev/tree/main#contributing) development environment.

## Releasing

This repository follows the **Semantic Versioning (SemVer 2.0.0)** standard:

> Given a version number **`MAJOR.MINOR.PATCH`**, increment the:
>
> - **MAJOR** version when you make incompatible API changes
> - **MINOR** version when you add functionality in a backward compatible manner
> - **PATCH** version when you make backward compatible bug fixes

- **Update Package Version:** Before tagging, update the `<version>` in `package.xml` (e.g., `1.2.3`). Commit and push your updates.

- **Tag and Push:** Create and push the new version tag (e.g., `v1.2.3`):

  ```bash
  git tag v1.2.3
  git push origin v1.2.3
  ```

  Pushing the tag will automatically publish a GitHub Release with auto-generated notes.

## Citations

If you use this repository in your research, please cite the following publications:

### CoUGARs

```bibtex
@misc{durrant2025lowcostmultiagentfleetacoustic,
  title={Low-cost Multi-agent Fleet for Acoustic Cooperative Localization Research},
  author={Nelson Durrant and Braden Meyers and Matthew McMurray and Clayton Smith and Brighton Anderson and Tristan Hodgins and Kalliyan Velasco and Joshua G. Mangelson},
  year={2025},
  eprint={2511.08822},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2511.08822},
}
```
