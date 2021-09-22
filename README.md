# Sampling Based Search Algorithms Implementation

## Overview

Implementation of Discerete Search Algorithm **Weighted A*** **(WA*)** and Sampling based Search Algorithm **Informed RRT***. 


Files included:

**PRM.py** includes a PRM class with four different sampling methods.

**RRT.py** includes a RRT class for RRT and RRT*.

**main.py** is the script that provides helper functions that load the map from an image and call the classes and functions from **PRM.py** and **RRT.py**.

**WPI_map.jpg** is a binary WPI map image with school buildings. You could replace it with some other maps you prefer.

## Instruction to Run the Code 
- Gitclone the repository.
- Add a new binary image file as a map if you want to try out new map and update the start and goal coordinates for path planning.
- Run the main.py script to see the plotted results of all 4 PRM sampling methods, RRT and RRT*.

## Results 

**Output of PRM**

<img width="400" height="400" alt="Uniform" src="https://user-images.githubusercontent.com/81267080/134376157-5782ac43-b679-4419-8503-ba6a3aa50daf.png">        <img width="400" height="400" alt="Random" src="https://user-images.githubusercontent.com/81267080/134380156-c1da62b3-63e2-4006-9045-33f87c1e9668.png">
<img width="400" height="400" alt="Gaussian" src="https://user-images.githubusercontent.com/81267080/134377922-226646fd-069a-4d90-9bd3-a90a7418462a.png">       <img width="400" height="400" alt="Bridge" src="https://user-images.githubusercontent.com/81267080/134375270-2b48d58f-ae95-46e4-af61-ad393e1340de.png">

**Output of RRT/RRT***

<img width="400" height="400" alt="RRT" src="https://user-images.githubusercontent.com/81267080/134381285-a0ee23dd-b236-4ca9-881b-b24f8b630337.png">       <img width="400" height="400" alt="RRT*" src="https://user-images.githubusercontent.com/81267080/134381334-f339b815-772f-4fa8-8eda-d8e2652eebd0.png">
