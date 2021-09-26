# hri_robot_arm

## Manual start without HoloLens2
If necessary, adjust the box coordinates in [CommandlineActionCall.bagy](misc/CommandlineActionCall.bagy). Then call
```bash
rostopic pub -f misc/CommandlineActionCall.bagy /hri_robot_arm/Record/goal hri_robot_arm/RecordActionGoal --once
```

## Coordinate systems and transformations
The following coordinate systems are commonly used:
![CordinateSystems](https://miro.medium.com/max/2000/1*NmzBmBk5SMUULxk8hcOsdw.png)
The realsense uses the OpenCV coordinate system. This is also the frame of COLMAP.

During recording, the transformation from camera to box/world is stored.
To convert the transformation to the position/pose, you can proceed as follows.

### World-to-camera transformation
```
x_local = R * x_world + t
R^t x_local = R^tR * x_world + R^t t
x_world = R^t (x_local - t)
```
That means the position of the camera (`x_local = (0,0,0)`) is:
```
x_world = - R^t * t
```

### Camera-to-world transformation
This is the transformation that we store during the recording.
Here the transformation already matches the pose of the camera, since
```
x_world = R' * x_local + t'
```
is `x_world = t'` for `x_local = (0,0,0)`.