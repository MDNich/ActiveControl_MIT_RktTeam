# Active Stabilization Development Roadmap
## Process
1. Choose a control surface configuration
2. Choose the degrees of freedom to use\
2a. Investigate 'spin can' for use with wings/canards
3. Choose a rough rocket dimension sketch
4. Develop a simulator to test control algorithm
5. Use simulator to refine algorithm parameters
6. Manufacture for use on a real rocket
7. (Ideally) Test in a wind tunnel.

## Control Surfaces

See the handwritten notes or (soon-to-be) LaTeX document describing control surface variants and their respective DOFs.

Factors to take into consideration:
1. Induced turbulences
2. Torque required for the servos
3. Shearing off of canards/wings should they be employed
4. Instabilities induced by the addition of canards/wings.

### Spin Can
Presented at the NAR by Jim Jarvis at vNARCON 2023, the [Spin Can](https://www.youtube.com/watch?v=00kFDRk2Cwg) can be used to mitigate effects of turbulence (?) - investigation with CFD required.

## Choosing a toy rocket
For now, Aurora's airframe will be used as a reference. If Polaris's airframe becomes usable, we will switch to that one.

## Simulator Development

See the folder named 'process'.

## Manufacturing

Should be relatively simple, as development of the whole control subsystem should have eliminated most manufacturing-unfriendly aspects of the design.

## Wind Tunnel 

A Wind Tunnel test (or several) may prove more beneficial than the simulator, or at least as beneficial, as it would permit live testing of the controls without the need for a fast computer, or a long wait time.
