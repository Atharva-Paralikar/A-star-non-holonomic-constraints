import numpy as np
import cv2
import math
import csv

class Node:
	def __init__(self,data,parent,curr_cost,heu_cost,rpm_set):
		self.Node_data = data 			# data = [x,y,angle]
		self.Node_parent = parent
		self.Node_curr_cost = curr_cost
		self.Node_heuristic_cost = heu_cost
		self.Node_action = rpm_set

def obstacle_check(x,y):

	global clearance
	global radius

	#Borders
	if 0 < x < (radius + clearance):
		if 0 < y < 1000:
			return True
	if  1000 - (radius + clearance) < x < 1000:
		if 0 < y < 1000:
			return True
	if 0 < x < 1000:
		if 1000 - (radius + clearance) < y < 1000:
			return True
	if  0 < x < 1000:
		if 0 < y < (radius + clearance):
			return True
	#Quadrilaterals
	if 25 - (radius + clearance) <= x <= 175 + (radius + clearance):
		if 425 - (radius + clearance) <= y <= 575 + (radius + clearance):
			return True
	if 375 - (radius + clearance) <= x <= 625 + (radius + clearance):
		if 425 - (radius + clearance) <= y <= 575 + (radius + clearance):
			return True
	if 725 - (radius + clearance) <= x <= 875 + (radius + clearance):
		if 200 - (radius + clearance) <= y <= 400 + (radius + clearance):
			return True
	#Circles
	if (x - 200)**2 + (y - 200)**2 <= (100 + radius + clearance)**2:
		return True
	if (x - 200)**2 + (y - 800)**2 <= (100 + radius + clearance)**2:
		return True
	else:
		return False

def get_obstacle_coord():

	mat_img = np.ones((1000,1000,3))
	mat = np.ones((1000,1000))

	for x in range(0,1001):
		for y in range(0,1001):
			if (obstacle_check(x,y)):
				mat_img[x -1][y-1] = (0,0,0)
				mat[x-1][y-1] = 0
	image = cv2.rotate(np.array(mat_img), cv2.ROTATE_90_COUNTERCLOCKWISE)
	image = cv2.convertScaleAbs(image, alpha=(255.0))
	cv2.imwrite("./obstacle_map.png",image)

	return mat_img,mat

def euclidean_distance(A,B):
	x1,y1,x2,y2 = A[0],A[1],B[0],B[1]
	dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
	return dist

def plot_curve(rpm,point,matrix_image):

	global counter
	l = 16																	## wheelbase
	r = 3.3																	## wheel radius
	t = 0
	dt = 0.05
	L_RPM, R_RPM = rpm[0],rpm[1]
	x_i, y_i, theta_i = point[0],point[1],(3.14 * point[2])/180
	ul = r * L_RPM * 0.10472
	ur = r * R_RPM * 0.10472
	start_point = (y_i,x_i)
	while (t < time_run):
		t = t + dt
		x_i += int((r/2) * (ul+ur) * math.cos(theta_i) * dt)
		y_i += int((r/2) * (ul+ur) * math.sin(theta_i) * dt)
		theta_i += (r/l) * (ur - ul) * dt
		image = cv2.line(matrix_image,start_point,(y_i,x_i),(0,0,255),2)
		image = cv2.convertScaleAbs(image, alpha=(255.0))
		cv2.imwrite("./frames/"+str(counter)+".png",image)
		start_point = (y_i,x_i)

def action(rpm,curr_node,Visited_region,goal,matrix_image):

	global counter
	l = 16																	## wheelbase
	r = 3.3																	## wheel radius
	t = 0
	dt = 0.05
	L_RPM, R_RPM = rpm[0],rpm[1] 
	x_i, y_i, theta_i = curr_node.Node_data[0], curr_node.Node_data[1], (3.14*curr_node.Node_data[2])/180
	
	ul = r * L_RPM * 0.10472
	ur = r * R_RPM * 0.10472
	avg_vel = int((ul + ur)/2)
	start_point = (y_i,x_i)
	while (t < time_run):
		t = t + dt
		x_i += int((r/2) * (ul+ur) * math.cos(theta_i) * dt)
		y_i += int((r/2) * (ul+ur) * math.sin(theta_i) * dt)
		theta_i += (r/l) * (ur - ul) * dt
		if (obstacle_check(x_i,y_i)) or (x_i < 0) or (y_i < 0):
			return False
		image = cv2.line(matrix_image,start_point,(y_i,x_i),(255,0,0),1)
		start_point = (y_i,x_i)
		Visited_region[x_i][y_i] = 1
	# image = cv2.line(matrix_image,start_point,end_point,colour,1)
	image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
	image = cv2.convertScaleAbs(image, alpha=(255.0))
	cv2.imwrite("./frames/"+str(counter)+".png",image)
	counter +=1
	cv2.imshow("frames",image)
	cv2.waitKey(1)
	
	final_orientation = 15 * round((180 * (theta_i) / 3.14)/15)
	start = [x_i,y_i]
	goal_dist = euclidean_distance(start,goal)
	nodeData = [x_i,y_i,final_orientation]
	nodeCurrCost = curr_node.Node_curr_cost + avg_vel
	nodeHueCost = goal_dist
	new_Node = Node(nodeData,curr_node,nodeCurrCost,nodeHueCost,rpm)
	return new_Node

def generate_path(node,matrix_image):     ## Bactracking function and generates the path

	global counter
	path = []
	path = [node]
	while node.Node_parent != None:
		node = node.Node_parent
		path.append(node)
	path.reverse()
	waypoints = []
	actionset = []
	for point in path:
		waypoints.append(point.Node_data[0])
		actionset.append(point.Node_action)
	actionset.pop(0)
	actionset.append([0,0])

	for i in range(len(waypoints)):
		plot_curve(actionset[i],waypoints[i],matrix_image)
	image = cv2.rotate(matrix_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
	image = cv2.convertScaleAbs(image, alpha=(255.0))
	cv2.imshow("final",image)
	for i in range(25):
		cv2.imwrite("./frames/"+str(counter)+".png",image)
		counter += 1
	cv2.waitKey(1000)

	with open("waypoints.csv", "w") as f:
		wr = csv.writer(f)
		wr.writerows(waypoints)

def videowrite():
	global counter

	image = cv2.imread("./frames/0.png")
	h,w,l = image.shape
	size = (w,h)
	video = cv2.VideoWriter("./video.mp4",cv2.VideoWriter_fourcc(*'mp4v'),30,size)
	print("generating video...")
	img = cv2.imread("./obstacle_map.png")
	video.write(img)
	for i in range(counter):
		img = cv2.imread("./frames/"+str(i)+".png")
		video.write(img)
	video.release()

def astar(start,goal,obstacles_mat,rpm):

	queue = []
	cost = []
	state = []
	Visited_region = obstacles_mat.copy()

	goal_dist = euclidean_distance([start[0],start[1]],[goal[0],goal[1]])
	startNode = Node(start,None,0,goal_dist,[0,0])
	queue.append(startNode)
	cost.append(startNode.Node_heuristic_cost)
	state.append([startNode.Node_data[0],startNode.Node_data[1]])

	while queue:
		minimum_cost = min(cost)
		k = cost.index(minimum_cost)
		# print("in queue")

		curr_node = queue.pop(k)
		cost.pop(k)
		state.pop(k)
		Visited_region[int(curr_node.Node_data[0])][int(curr_node.Node_data[1])]= 1

		if goal_dist >= 10:
			for rpm in action_space:
				new_Node = action(rpm,curr_node,Visited_region,goal,matrix_image)
				
				if new_Node != False:
					# print(new_Node.Node_heuristic_cost,new_Node.Node_data[2])
					goal_dist = euclidean_distance([new_Node.Node_data[0],new_Node.Node_data[1]],goal)
					if goal_dist <= 20:
						print("Goal reached!")
						return new_Node,matrix_image
					else:
						pos = [new_Node.Node_data[0], new_Node.Node_data[1]]
						if pos in state:
							idx = state.index(new_Node.Node_data)
							if new_Node.Node_heuristic_cost < queue[idx].Node_heuristic_cost:
								queue[idx].Node_parent = new_Node.Node_parent
								queue[idx].Node_heuristic_cost = new_Node.Node_heuristic_cost
						else:
							queue.append(new_Node)
							cost.append(new_Node.Node_heuristic_cost)
							state.append(new_Node.Node_data)
							Visited_region[int(new_Node.Node_data[0])][int(new_Node.Node_data[1])] = 1
							# print(new_Node.Node_heuristic_cost)
	print("Goal Unable to reach at given orientation")
	
if __name__ == '__main__':

	flag = True

	radius = 21
	time_run = 1
	counter = 0

	while (flag):
		start_x = int(input("Enter Start position X between (0,1000): "))
		start_y = int(input("Enter Start position Y between (0,1000): "))
		theta_s = int(input("Enter Start orintation theta_s (...,-30,-60,0,30,-60,...): "))
		goal_x = int(input("Enter Goal position X between (0,1000): "))
		goal_y = int(input("Enter Goal position Y between (0,1000): "))
		RPM1 = float(input("Enter Wheel RPM 1: "))
		RPM2 = float(input("Enter Wheel RPM 2: "))
		clearance = int(input("Robot Clearance: "))
		matrix_image,obstacle_mat = get_obstacle_coord()

		start = [start_x,start_y,theta_s]
		goal = [goal_x,goal_y]
		action_space = [[0,RPM1],[RPM1,0],[RPM1,RPM1],[0,RPM2],[RPM2,0],[RPM2,RPM2],[RPM1,RPM2],[RPM2,RPM1]]

		if obstacle_mat[start_x][start_y] == 0 or obstacle_mat[goal_x][goal_y] == 0:
			print("Start position or Goal position in obstacle space. Enter the configuration again !")
		else:
			final,image = astar(start,goal,obstacle_mat,action_space)
			generate_path(final,image)
			videowrite()
			flag = False