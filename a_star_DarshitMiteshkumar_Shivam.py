# import the pygame module
import pygame as pyg
import numpy as np
#import queue module
from queue import PriorityQueue
import math
import time
from geometry_msgs.msg import Twist



# Function to calculate distance between two points

# Function that appends the generated nodes to the open list if node not in open list
# If the list is in open list it updates the open list
# hello its me

 
# To input the step size from the user and validate the step size

#Turtlebot3 burger parameters
radii = 0.105*100 #Turtlebot3 radius
white = (255,255,255)
# Taking input from user for clearance and radius of robot and defining the canvas
print("ROBOT CLEARANCE DIMENSIONS AND RADIUS(Radius is fixed). Enter valid dimensions between 0 to 50")
while (True):
    clr = int(input("Enter the clearance of the robot: "))
    if ((clr>0 and clr<50)):
        print("Valid coordinates received")
        #Define the Surface Map
        screen = pyg.Surface((600, 200))
        #Define the rectangles which make the base map
        rect_color = (255, 255, 255)
        #Define the rectangle which makes the outer border
        rectangle1 = pyg.Rect(clr+radii, clr+radii, 600-2*(clr+radii), 200-2*(clr+radii))
        screen.fill((255,0,0))
        pyg.draw.rect(screen, rect_color, rectangle1)
        #Define the rectangle which makes the 2 rectangles
        bottom_rect_dim = [(250-radii-clr,200),(265+radii+clr,200),(265+radii+clr,75-radii-clr),(250-radii-clr,75-radii-clr)]
        pyg.draw.polygon(screen, (255,0,0),bottom_rect_dim)
        top_rect_dim = [(150-radii-clr,0),(165+radii+clr,0),(165+radii+clr,125+radii+clr),(150-radii-clr,125+radii+clr)]
        pyg.draw.polygon(screen,(255,0,0),top_rect_dim)
        pyg.draw.circle(screen, (255,0,0,),(400,90),50+radii+clr)
        break
    else:
        print("Invalid coordinates received, Try again")
        continue

# Taking start position and goal position for the robot from the user and validating them
while True:
    try:
        print("Enter Robot start and goal coordinates. Ensure that theta values are multiples of 30 deg")
        start_x=int(input("Enter the starting x coordinate: "))    
        start_y=int(input("Enter the starting y coordinate: "))   
        start_theta=int(input("Enter the start theta orientation: ")) 
        if (start_theta%30!=0):
            print("Invalid Theta value try entering the coordinates again")
            continue
        goal_x=int(input("Enter the goal x coordinate:"))    
        goal_y=int(input("Enter the goal y coordinate: ")) 
        goal_theta=int(input("Enter the goal theta orientation: \n"))
        if (goal_theta%30!=0):
            print("Invalid Theta value try entering the coordinates again")
            continue
        start_y=200-start_y
        goal_y=200-goal_y
        robot_start_position=(start_x,start_y)
        robot_goal_position=(goal_x,goal_y)
        if screen.get_at(robot_start_position) != white or screen.get_at(robot_goal_position)!=white:
            raise ValueError
        break
    except ValueError:
        print("Wrong input entered. Please enter an integer in correct range x(0,599) and y(0,249).")        

# Take robot wheel RPMs
while True:
    try:
        print("Enter the Robot RPM sets")
        RPM1 = float(input("Enter Robot wheel RPM 1: "))
        RPM2 = float(input("Enter Robot wheel RPM 2: "))
        if (RPM1<0 or RPM2<0):
            print("Negative values not allowed. Enter valid values of RPM")
            continue
        break
    except ValueError:
        print("Wrong input detected, Enter again")


def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    L = 0.160*100 #Turtlebot 3 wheel distance
    r = 0.033*100 #Robot wheel radius
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180


# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    points=[(Xn,Yn)]
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        points.append((round(Xn),round(Yn)))
    Thetan = 180 * (Thetan) / 3.14
    
    return Xn, Yn, Thetan, D, points

# for action in actions:
#      k=cost(0,0,45, action[0],action[1])      # (0,0,45) hypothetical start configuration, this dosn't matter for calucating the edges'costs
#      print(k[3])


# Function that appends all the exploration nodes to a list new_nodes
def move_robot(robot,curr_theta, costtocome):
    x,y=robot
    new_nodes = []
    actions = [[0,RPM1],[RPM1,0],[RPM1,RPM1],[0, RPM2],[RPM2, 0],[RPM2, RPM2],[RPM1, RPM2],[RPM2, RPM1]]
    for action in actions:
        x_t,y_t,t_t,c2c,points =  cost(x,y,curr_theta,action[0],action[1])
        c2g=math.dist((x_t,y_t),(goal_x,goal_y))
        robot_position=(round(x_t),round(y_t))
        total_cost_come=c2c+costtocome
        new_nodes.append([c2g+total_cost_come,total_cost_come,c2g,robot_position,t_t,points,action])
    return new_nodes

def new_node(new_node_list):
    total_cost=new_node_list[0]
    cost_to_come=new_node_list[1]
    cost_to_goal=new_node_list[2]
    new_pos=new_node_list[3]
    t=new_node_list[4]
    x,y=new_pos 
    points=new_node_list[5]
    r_p_m = new_node_list[6]
    print("current rpm",r_p_m)
    if (((x>0 and x<600) and (y>0 and y<200))==True):
        if ( not(any(screen.get_at((a,b))!=white for a,b in points)) and screen.get_at((new_pos)) == white and not (new_pos in check_closed_list)):
            if not (new_pos in global_dict):
                global node_index
                node_index += 1
                global_dict[new_pos]=[total_cost,cost_to_come,cost_to_goal,node_index,info[3],new_pos,t,points,r_p_m]
                open_list.put(global_dict[new_pos])
                dict_vector[info[5]]=points
            else:
                if (global_dict[new_pos][1]>cost_to_come):
                    global_dict[new_pos][4]=info[3]
                    global_dict[new_pos][1]=cost_to_come
                    global_dict[new_pos][0]=total_cost
                    global_dict[new_pos][6]=t
                    global_dict[new_pos][7]=points
                    global_dict[new_pos][8]=r_p_m

ctc_node=0  # cost to come for start node
ctc_goal=math.dist((start_x,start_y),(goal_x,goal_y)) # cost to goal for the start node
parent_node_index=None # Index for the parent node
node_index=0 # Index of the current node
closed_list={} # dictionary to store information about the current node
check_closed_list={} # dictionary to store the nodes to check if nodes present in closed list
open_list=PriorityQueue() # list the store nodes and pop them according to priority
rp_m = [0,0]
pts = []
info=[ctc_goal+ctc_node,ctc_node,ctc_goal,node_index,parent_node_index,robot_start_position,start_theta,pts,rp_m] # list to save all info of a node
open_list.put(info)
global_dict={} # global dictionary to reference all the information for nodes in the open list and to update the information
global_dict[robot_start_position]=[ctc_goal+ctc_node,ctc_node,ctc_goal,node_index,parent_node_index,robot_start_position,start_theta,pts,rp_m]


start_time=time.time()# to store start time of the algorithm
end_loop=0 # variable to break out of the loop
dict_vector = {} # to save the node as key and it childs as values to draw nodes as vectors

# loop to explore the nodes and find the goal
while True and end_loop!=1:
    # if the open list is empty means that no solution could be found
    if(open_list.empty()):
        print("No solution")
        goal_node=None
        break
    
    info=open_list.get()
    
    new_nodes=move_robot(info[5],info[6],info[1])
    # append the node to node list                                               
    for i in range(0,8):
        if(new_nodes[i][2]<=0.5):
            print("goal reached")
            print("I breaked at: ",i)
            closed_list[node_index+i+1]=[new_nodes[i][0]+info[1],new_nodes[i][1]+info[1],new_nodes[i][2],info[3],new_nodes[i][3],new_nodes[i][4],new_nodes[i][-1]]
            global_dict[new_nodes[i][3]]=[new_nodes[i][0],new_nodes[i][1],new_nodes[i][2],node_index+i+1,info[3],new_nodes[i][3],new_nodes[i][4],new_nodes[i][5],new_nodes[i][-1]]
            goal_node=node_index+i+1
            end_loop=1
            break 
        new_node(new_nodes[i])
                                    
    closed_list[info[3]]=[info[0],info[1],info[2],info[4],info[5],info[6],info[7]]
    print(closed_list[info[3]])
    check_closed_list[info[5]]=None
    

green=(0,255,0) # color for backtracking path line
end_time=time.time() # to store end time for algorithm
print("Total time taken for search:",end_time-start_time) # to check the total time taken for th algorithm

screen_display = pyg.display.set_mode((600, 200)) # Create a screen
screen_display.blit(screen, (0, 0))
pyg.display.update()


# Find the path form start to goal
path = []
path_theta_list = []
RPM_list = []
if goal_node!=None:
    st_time = time.time()
    print("The final goal node is given by: ",goal_node)
    while goal_node is not None:
        goal_node_parent = closed_list[goal_node][3]
        path.append(closed_list[goal_node][4])
        path_theta_list.append(closed_list[goal_node][5])
        RPM_list.append(closed_list[goal_node][6])
        goal_node=goal_node_parent        
    # reverse the path list to get the correct order of nodes
    path.reverse()
    path_theta_list.reverse()
    RPM_list.reverse()
    et_time = time.time()
    print("Total time taken for backtracking:",et_time-st_time)
    print("****** The optimum path is ****",path)

# To draw the graph of exploration nodes on the canvas
for key in dict_vector.keys():
    pyg.draw.lines(screen_display,(0,0,0),False,dict_vector[key])
    pyg.display.update()

print("Length of closed nodes=",len(closed_list)) 
path.pop(0)
# x=path.pop(0)
# To draw the path taken by the robot from start node to goal node
for i in range(len(path)):
    # if i+1>len(path):
    #     break
    print(global_dict[path[i]])
    pyg.draw.lines(screen_display,(0,255,0),False,global_dict[path[i]][7])

    pyg.display.update()
# for key, value in closed_list.items():
#     print(f"{key}: {value}\n")
# while y!=0:
#     print(global_dict[x][7])
#     pyg.draw.lines(screen_display,(255,0,0),False,global_dict[y][7])

#     pyg.display.update()
    
# for key, value in closed_list.items():
#     print(f"{key}: {value}")
# Set the caption of the screen
print("RPM list length",len(RPM_list))
print(RPM_list)
pyg.display.set_caption('A* Visualization Map')
pyg.display.update()
pyg.time.wait(1)
running=True
while running:
	# for loop through the event queue
	for event in pyg.event.get():
		# Check for QUIT event	
		if event.type == pyg.QUIT:
			running = False

for i in range(0,len(path)):
    xt,yt = path[i]
    yf = 200-yt
    yf1 = yf -100
    xf = xt-50
    xf1 = xf/100
    yf2 = yf1/100
    path[i]=(xf1,yf2)
print("I'm done till here")


import rospy

rospy.init_node('ROS_AStar', anonymous=True)
pub_vel = rospy.Publisher('cmd_vel',Twist,queue_size=100)
msg = Twist()
turtle_model = rospy.get_param("model","burger")
msg.linear.x = 0.0
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 0.0

pub_vel.publish(msg)
r = rospy.Rate(10)
t =1 #Time step
for i in range(len(path)):
    x1,y1 = path[i] 
    x2,y2 = path[i+1]
    t1 = path_theta_list[i]
    t2 = path_theta_list[i+1]
    dx = x2-x1
    dy = y2-y1
    vx = dx/t
    vy = dy/t
    dw = t2-t1
    wt = dw/t
    vel_inp = math.sqrt(vx**2+vy**2)
    msg.linear.x = vel_inp
    msg.angular.z = wt
    pub_vel.publish(msg)
    r.sleep()





