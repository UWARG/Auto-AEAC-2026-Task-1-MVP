# Auto Task 1 MVP system

Vibe coded by Claude Code, based from <https://github.com/UWARG/AEAC-2025-task-1-auto-centre>
All code is WIP and is subject to review/changes

## Drone setup/assumptions

- There are 2 cameras, on facing forward, one facing down

## Mission Plan

- Mapping the building
  - fly to building
  - pilot manually flies to each corner of the building
  - when directly above a corner, flip switch to record coordinate
    - directly above is according to the pilot's judgement
  - also need to triage getting building height
  - see ` building_mapper.py` for implementation
- Targets on the ground
  - same logic as [last year's script](https://github.com/UWARG/AEAC-2025-task-1-auto-centre)
  - only meaningful difference is that CV looks for colors
- Targets on the roof
  - same logic as targets on the ground
  - if target is withing the bounds of the building,
    adjust altitude/description accordingly
- Targets on the wall
  - needs new auto centering logic
    - current code doesn't support adjusting drone altitude
  - TODO: calculate coordinate of target from detection
    - draw straight line from drone, calculate where it intersects with the closest face of the building
  - altitude is taken from the drone
    - this may be a problem if the target is low to the ground
