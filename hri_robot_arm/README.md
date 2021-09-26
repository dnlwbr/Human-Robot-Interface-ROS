# hri_robot_arm

## Manual start without HoloLens2
If necessary, adjust the box coordinates in [CommandlineActionCall.bagy](misc/CommandlineActionCall.bagy). Then call
```bash
rostopic pub -f misc/CommandlineActionCall.bagy /hri_robot_arm/Record/goal hri_robot_arm/RecordActionGoal --once
```