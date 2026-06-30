# Architecture

This document outlines the architecture of the LEAP Sprayer software stack,
especially what the component ROS nodes are and what their function is.

This document uses Mermaid diagrams to visualize parts of the architecture.
It is best viewed on GitHub or in an editor that supports Mermaid diagrams,
although Mermaid code is readable enough to understand without support.

## Detection Stack

The detection stack takes in camera images and robot motion to detect and track
weeds while driving.

```mermaid
graph LR
  tf_cam([Camera Transform])
  velo_robot([Robot Velocity])

  cam[Camera]
  
  subgraph Weed Detection
    direction LR
    seg[Segmentation]
    bound[3D Bounding]
    seg -->|weed_detection/segment_mask| bound
  end
  cam -->|camera/color/image| seg
  cam -->|camera/depth/image| bound
  cam -->|camera/cam_info| bound
  tf_cam -->|tf_static/camera| bound

  track[Track In 3D]
  bound -->|weed_detection/bbox| track
  tf_cam -->|tf_static/camera| track
  velo_robot -->|odom/velocity| track
```

### The Camera

The camera node publishes an image and a depth image.
This allows for supporting depth cameras to allow for more accuracy.
For monocular cameras, a decent assumption can be made by making
the depth image correspond to the distance to a flat floor plane.
If monocular needs to be used with better depth accuracy,
something like Depth Anything could estimate depth,
and thanks to the composability of the architecture,
it doesn't matter as long as depth images that correspond with a color image
share a timestamp to indicate this correspondence.

### Weed Detection

Weed detection could possibly be more complex than the above diagram,
using data like the depth image to detect more accurately.
So long as the output is a 3D bounding box relative to the robot frame,
this implementation can remain opaque to the rest of the system.

As of right now, our explored approaches utilize just color images
when detecting and segmenting weeds, so by adding the depth information
separately, these pipelines can remain unaware of the depth information.
The 3D Bounding node will just use the segmented pixels and their corresponding depth values to estimate the bounding box.

### Track In 3D

This node ingests the 3D bounding boxes found on each frame
and uses knowledge of past detections to extrapolate the positions of weeds
as the robot moves, even as weeds may move out of view
(notably, they may leave view before reaching the sprayer).

This step also helps denoise the data using its knowledge of expected positions
and the robot's velocity.
