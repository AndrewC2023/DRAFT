# DRAFT: Dynamics, Routing, Autonomy, and Feasibility Testbed

A platform for dynamics simulation and autonomy algorithms meant to be paired with POUR for sensor modeling, and BREW for tracking and navigation. DRAFT alone provides a testbed for autonomy, planning, control, and guidance testing with dynamical systems, and together with POUR and BREW provides a space for modeling and simulation of a full autonomy stack. 

- Draft is built on work done at the University of Alabama through the LAGER repo GNCpy and planning and guiadnce work done in relation to the Astrobotics team and research conducted at LAGER.  

## DRAFT goals
It is the desire for DRAFT to have a Qt based frontend eventually allowing users to setup systems there or more directly. On the backend side we hope to offer both fully in house dynamics, filtering, autonomy, and perception but with the option to use potentiall more well known backends, currently gazeebo is the primary alternative backend being explored. 

## Development Environment Setup
It is recommended for development to use the dockerfile provided to build a container and run tests validation and build documentation in there. The dev container should ease the process of building individual modules of DRAFT and test them separatesly with the unit tests or build validation scripts to put together a sim without the use of the frontend. For now front end is not supported and there are not plans to support it in the backend dev container.

To setup the Dev Container you must have docker on your machine which likely will need wsl installed as well. Make sure the docker daemon is running before building or running the image/container. To do this you can run docker desktop on windows. On linux machines one may have to setup the docker daemon to run on system start or start running it from the terminal. 

For first time setup you will need to build the image (note you should run these commands from the root directory of the repository):
```powershell
docker build -t draft-dev:latest .
```
Once the image is built you can run the following to open a container:
- Windows:
```powershell
docker run --rm -it --name draft-container --mount "type=bind,source=$((Get-Location).Path),target=/workspace" draft-dev:latest
```
- Bash:
```bash
docker run --rm -it --name draft-container --mount type=bind,source="$(pwd)",target=/workspace draft-dev:latest
```

just type ```exit``` in the container to close it, for subsequent use the ducker run commands above just need to be run from the root directory so long as you have need deleted the image otherwise it must also be rebuilt.
<!-- note that if I eventually host the image building it wont be needed -->
## STOUT add-on
STOUT: or Stochastic Trials for Optimal Uncertainty-aware Trajectories is a project on using uncertain dynamics systems for path planning and guidance. Draft in its autonomy framework will eventually be able to support this and may in the end fully incorporate STOUT.

## BREW add-on

## POUR add-on

## Contribution
At the moment for contribution, ask Andrew Campbell for access if you do not already have it, then checkout a branch from master named approapraitely for the feature you want to add. develop on that branch, write tests and document please. Once done if possible please merge the parent branch/master branch back into yours to resolve conflicts if they exist, retest and submit a pull request.