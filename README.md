# Application to create a 3D model of a room from 2D images
Computer Vision Project - 3D reconstruction of a room from 2D images

![alt text](https://github.com/MarcelHa97/3D-Room-Reconstruction/blob/Projects/img/img_GUI.png)

# Installation
1. Download all files from the source folder and put it in one common folder
2. Run 'GUI_normal_new.mlapp' to start the App

# User Manual
1. Image and Camera Data Upload Buttons \
These buttons enable you to import external images or camera calibration data into the application.\
Select a filepath for the images and a .txt file for the camera data. 

2. Create 3D View \
After importing the images and camera calibration data, you can press this button and create the 3D view. 

3. Perspective Buttons \
The perspective buttons allow you to change the viewpoint of the 3D  reconstructed room. \
Each button corresponds to a specific perspective, namely 'Top', 'Right', 'Front' or 'Left'. \
When you click on a perspective button, the 3D plot will update to show the scene from the selected viewpoint. 

4. Slider for Room Exploration \
The slider in the GUI provides an interactive way to explore a virtual room. \
By adjusting the slider, you can change your angle within the room, simulating turning around and changing your viewing direction. \
To reset to home, press the reset camera button. 

5. Line and Plane Area Measurement Buttons \
The GUI offers two buttons specifically designed for measurement tasks. \ 
The line measurement button allows you to measure the length between two points in the room. \
The plane measurement button enables you to measure the area of a planar region in the plot. 
