# Basic searching algorithms
import numpy as np
from numpy import inf 
import math

def init(grid, start, goal):
    epsi=3                                              # CHANGE EPSILON TO CHANGE WEIGHT OF HEURISTIC
    path = []
    steps = 0
    found = False
    Row=len(grid)                                       # Initialise Row
    Col=len(grid[0])                                    # Initialise column 
    if start[0]==goal[0] and start[1]==goal[1]:         # Condition if start and goal are at same point 
        found=True
        steps=-1
    
    grid[goal[0]][goal[1]]="B"                          # Assigning B as goal name on the grid
    
    visited=[[False for j in range(Col)] for i in range(Row)]       # Creating bool false array of RxC to check if node has been visited
    visited[start[0]][start[1]]=True
    steps=steps+1
    prev = [[0 for i in range(Col)] for j in range(Row)]            # Array for storing parent node of the current node visted
    prev[start[0]][start[1]]=start
    cost=[[inf for i in range(Col)] for j in range(Row)]
    cost[start[0]][start[1]]=epsi*(abs(start[0]-goal[0])+abs(start[1]-goal[1]))
    queue=[]
    queue.append([0,start[0],start[1]])
    distance=0

    return path, steps, found, Row, Col, visited, prev,grid,epsi,cost,queue,distance

def cost1(point,start,prev):            # Cost manhattan cost to a point from start

    cost=0
    at=point
    while at!=start:
        cost=cost+1
        at = prev[at[0]][at[1]]
    
    return cost

def wastar(grid, start, goal):
    '''Return a path found by WA* alogirhm 
       and the number of steps it takes to find it.
    '''
    ### YOUR CODE HERE ###
    path, steps, found, Row, Col, visited, prev,grid,epsi,cost,queue,distance=init(grid, start, goal)
    while (len(queue))>0:

        if found==True:
            break

        temp=queue.pop(0)
        r=temp[1]
        c=temp[2]
        dr=[0,+1,0,-1]
        dc=[+1,0,-1,0]
        steps=steps+1
        for i in range(4):
            rr = r + dr[i]
            cc = c + dc[i]
            if rr<0 or cc<0:
                continue
            if rr>=Row or cc>=Col:
                continue
            if visited[rr][cc]==True:
                if cost[rr][cc]>cost1([rr,cc],start,prev) + epsi*(abs(rr-goal[0])+abs(cc-goal[1])):
                    cost[rr][cc]=cost1([rr,cc],start,prev) + epsi*(abs(rr-goal[0])+abs(cc-goal[1]))
                    prev[rr][cc]=[r,c]
                continue
            if grid[rr][cc]==1:
                continue
            prev[rr][cc] = [r,c]
            distance=cost1([rr,cc],start,prev)                  # cost to node g(x)
            heuristic=abs(rr-goal[0])+abs(cc-goal[1])           # h(x) as manhattan distance of node from goal 
            cost[rr][cc]=distance+epsi*heuristic                # calculate total cost using weighted heuristic
            queue.append([cost[rr][cc], rr,cc])
            visited[rr][cc]=True
            
            
            if grid[rr][cc]== "B":
                found=True
                break
        
        queue.sort()                                                    # sort queue based on total cost 
    at = [goal[0],goal[1]]
    counter=0
    while at!=start:
        path.append(at)
        at = prev[at[0]][at[1]]
        counter=counter+1
    path = path[::-1]

    if found:
        print(f"It takes {steps} steps to search for a path using WA*")
        print(f"It takes {counter} steps to find traverse path using WA*")
    else:
        print("No path found")
    return path, steps

