# Sampling Based Search Algorithms Implementation

## Overview

Implementation of Discerete Search Algorithm **Weighted-A*** (**WA***) and Sampling based Search Algorithm **Informed RRT***. 

The usual **A*** algorithm bases heuristic upon the manhattan distance to the goal as a bias but in **WA*** there is a higher bias given to a path in the general direction towards the goal by using a higher value of heuristic assigned through a weighted heuristic **epsilon > 1**

Files included:

**Informed RRT*** **Folder**

- **Informed_RRTstar.py** is the script that is included in the Informed RRT* folder. The same folder contains main.py which is the script used to load the helper functions and load the binary map.
- **WPI_map.jpg** is a binary WPI map image with school buildings. You could replace it with some other maps you prefer.

**WA***

- **Search.py** carries the implementatoin of the Weighted A* algorithm and is located within the WA* folder. The same folder has the main.py file associated with this search algorithm. 
- **Map.csv** is the file that is used as a map where 1s represent obstacles and 0s represent free space. 

## Instruction to Run the Code 
- Gitclone the repository.
- Run the main.py script in either of the folders to run the corresponding algorithms. 
- Change map files to use your own maps if you like.
- Modify number of nodes to be sampled in Informed RRT* and Episilon (weight for heuristic) to see different results. 

## Results 

**Output of regular A*** **using Epsilon = 1**

<img width="400" height="400" alt="A*" src="https://user-images.githubusercontent.com/81267080/134437325-a00e2657-fac6-4d37-815a-8f61d56d405e.png">   <img width="600" alt="A*Result" src="https://user-images.githubusercontent.com/81267080/134437683-6ae62954-f334-41b5-9d7b-6e60c206d3c5.png">

**Output of WA*** **using Epsilon = 3 and 6**

<img width="400" height="400" alt="WA*E3" src="https://user-images.githubusercontent.com/81267080/134437862-ca2386c6-d069-40e7-bbf5-0c4ca8334a50.png">       <img width="400" height="400" alt="WA*E6" src="https://user-images.githubusercontent.com/81267080/134437908-f8914cd6-554a-4c9c-b9bb-5253aea6cefc.png">

<img width="400" alt="WA*Er3" src="https://user-images.githubusercontent.com/81267080/134438063-50f3db6c-cf5c-483f-9201-c63d32234f85.png">    <img width="400" alt="WA*Er6" src="https://user-images.githubusercontent.com/81267080/134438036-76f250da-bd53-4fe9-9220-2b847a21a98a.png">  


**Output of Informed RRT*** **using 1000 and 2000 samples**

<img width="400" height="400" alt="WA*Er3" src="https://user-images.githubusercontent.com/81267080/134438462-d47e6d38-03c7-4b70-9e55-f1517ab26b13.png">     <img width="400" height="400" alt="WA*Er3" src="https://user-images.githubusercontent.com/81267080/134438515-3088272b-e880-45ee-be94-8e23ccd97f06.png">

<img width="400" alt="WA*Er3" src="https://user-images.githubusercontent.com/81267080/134439098-e368cb1a-1ab2-497e-86fe-848092f53b81.png">    <img width="400" alt="WA*Er6" src="https://user-images.githubusercontent.com/81267080/134439116-f4df0f3e-aaef-46a9-b96b-59e7efe9e25f.png">  

