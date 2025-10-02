# Auto Task 1 MVP system

Main Task 1 MVP

## Setup

- Set up and activate Python3.11 venv on drone and ground station
- Run `airside` on drone
  - TODO: setup autostart on drone
- Run `groundside` on ground control station

## Mission plan

- **Map the building**
  - manually fly over each of the 4 corners of the building
  - above a corner, flip switch X to record it
    - Recording will reset beyond 4 flips
  - manually fly level to the roof the building
  - flip switch Y to record building height
- **Detect Targets**
  - manually fly around the building until target is visible on Camera
  - if detect target on roof of building or ground
    - flip switch A to start auto-centring
    - flip switch A againt to stop auto-centring and record
  - if detect target on walls of the building
    - flip switch B to start auto-centring
    - flip switch B againt to stop auto-centring and record
