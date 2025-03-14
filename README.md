# ROS2 Bag To Image

This is a simple ROS2 node that opens bag files from `bags/`, looks for the topic `/camera/image_raw`, then saves the image to `/images` under the same bag name and image index. While completing this operation, the node will also publish the image to `/image` for real-time monitoring if necessary.

## Installation:

To install, run the following code:
