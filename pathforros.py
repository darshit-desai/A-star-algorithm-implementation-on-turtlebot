def move_robot_ros(path):
    dt =1
    r = 0.033*100
    L = 0.160*100
    for i,coord in enumerate(path):
        x1 = coord[i][0]
        y1 = coord[i][1]
        t1 = coord[i][2]
        x2 = coord[i+1][0] 
        y2 = coord[i+1][1]
        t2 = coord[i+1][2]
        dth = t2-t1
        dx = x2-x1
        dy = y2-y1



