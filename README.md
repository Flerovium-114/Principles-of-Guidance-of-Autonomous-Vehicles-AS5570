# Principles-of-Guidance-of-Autonomous-Vehicles-AS5570

This repository contains the codes for the basic guidance algorithms as well as the code used for the multiphase guidance law as part of the term project. 
Basic Guidance Laws:
- Pure Pursuit (PP)
- Deviated Pure Pursuit (DPP)
- Line of Sight Guidance: Beam Rider (BR) and Command-to-LOS (CLOS)
- True Proportional Navigation (TPN)
- Pure Proportional Navigation (PPN)

Term Project:
- Replicated the 2-D results of *’Unmanned Aerial Vehicle Guidance for an All-Aspect Approach to a Stationary Point'*, by S. Ghosh, O. A. Yakimenko and D. T. Davis.
- A multiphase guidance algorithm was developed which to attain the desired approach angle to a *stationary target* which also addressed the Field Of View constraints of the pursuer.
- An extension to the paper was presented which aimed at finding the **optimum Navigation constant** for the Orientation phase that minimized the control effort for the UAV in the phase.
- The various codes provided are needed to obtain the specific plots in the paper but all of them follows the same general algorithm.
