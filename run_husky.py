#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
import math
import tf



def get_path():
    xy = []

    xy.append([-0.5,29])
    xy.append([-70,29])
    xy.append([-0.5,29])
    xy.append([-0.5,93])
    xy.append([24,93])
    xy.append([-0.5,93])
    xy.append([-0.5,151.5])
    xy.append([27,151.5])
    xy.append([-0.5,151.5])
    xy.append([-50,151.5])
    xy.append([-103,151.5])
    xy.append([-41.5,151.5])

    # xy.append([-0.5,151.5])
    # xy.append([27,151.5])
    # xy.append([-103,151.5])
    # xy.append([-38,151.5])

    xy.append([-41.5,215])
    xy.append([-89,215])
    xy.append([60,215])
    xy.append([6,215])
    xy.append([6,284])

    # Make new list with swapped elements
    # new_xy = []
    # for i in xy:
    #     new_xy.append([i[1],i[0]])

    return xy


curr_pose = (None,None)
start_pos = True
heading = 0
def pos(data):
    global curr_pose
    curr_pose = (data.pose[-5].position.x,data.pose[-5].position.y)

    global heading
    quaternion = (data.pose[-5].orientation.x,data.pose[-5].orientation.y,data.pose[-5].orientation.z,data.pose[-5].orientation.w)
    r,p,y = tf.transformations.euler_from_quaternion(quaternion)
    
    # if y < 0:
    #     heading = 2*math.pi-abs(y)
    # else:
    #     heading = y
    heading = y

def get_direction(a):
    if a > 0 and heading < 0:
        if heading < -math.pi/2:
            dir = -1
            new_h = 2*math.pi-abs(heading)
            mag = abs(new_h-a)
        else:
            dir = 1
            mag = abs(a-heading)
    elif a < 0 and heading > 0:
        if a < -math.pi/2:
            dir = 1
            new_a = 2*math.pi-abs(a)
            mag = abs(new_a-heading)
        else:
            dir = -1
            mag = abs(heading-a)
    else:
        dir = 1 if (a-heading)>0 else -1
        mag = abs(a-heading)

    return dir,mag   



def euc_dist(p1,p2):
    return math.sqrt((p2[1]-p1[1])**2+(p2[0]-p1[0])**2)

def controller():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.init_node('astar_driver', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, pos)
    start = (None,None)
    rate = rospy.Rate(15)
    rotating_flag = False
    valid_rotation = 99
    while curr_pose == (None,None):
        pass


    start = curr_pose
    astar_path = get_path()
    print(astar_path)
    if astar_path:
        
        disp_list = []
        for i in astar_path:
            disp_list.append((i[1],i[0]))

        # rospy.loginfo(disp_list)
        while not rospy.is_shutdown():
            print("Path Exists")
            try:
                inter_goal = astar_path.pop(0)
                while euc_dist(curr_pose,inter_goal) > 0.4:
                    angle = math.atan2((inter_goal[1]-curr_pose[1]),(inter_goal[0]-curr_pose[0]))
                    # if angle < 0:
                    #     angle = 2*math.pi-abs(angle)

                    twist_msg = Twist()
                    rot_dir,magnitude = get_direction(angle)
                    
                    while magnitude>6*math.pi/180:
                        # print("LTURNING:",math.degrees(angle),math.degrees(heading),heading-angle)
                        # print("LF",round(curr_pose[0],4),round(curr_pose[1],4),round(inter_goal[0],4),round(inter_goal[1],4),round(euc_dist(curr_pose,inter_goal),4),'\n')
                        print("XXXXX:",math.degrees(angle),math.degrees(heading),math.degrees(magnitude))
                        if not rotating_flag:
                            rotating_flag = True
                            rot_dir,magnitude = get_direction(angle)
                            print("M:",rot_dir,magnitude)
                            if magnitude < 0.3:
                                twist_msg.angular.z = 0.3*rot_dir
                            else:
                                twist_msg.angular.z = 0.3*rot_dir*magnitude
                        twist_msg.linear.x = 0
                        pub.publish(twist_msg)
                        rate.sleep()
                        angle = math.atan2((inter_goal[1]-curr_pose[1]),(inter_goal[0]-curr_pose[0]))
                        _,magnitude = get_direction(angle)
                    print("F",round(curr_pose[0],4),round(curr_pose[1],4),round(inter_goal[0],4),round(inter_goal[1],4),round(euc_dist(curr_pose,inter_goal),4))

                    rotating_flag = False
                    
                    angle = math.atan2((inter_goal[1]-curr_pose[1]),(inter_goal[0]-curr_pose[0]))

                    if euc_dist(curr_pose,inter_goal) > 5:
                        twist_msg.linear.x = 1
                    else:
                        twist_msg.linear.x = 0.4

                    rot_dir_2,mag_2 = get_direction(angle)
                    if mag_2 < 0.02:
                        twist_msg.angular.z = 0.0
                    else:
                        twist_msg.angular.z = 0.02*rot_dir_2
                    # print("PPPPP:",math.degrees(angle),math.degrees(heading),math.degrees(magnitude))
                    # print("small turn",twist_msg.angular.z,'\n')
                    pub.publish(twist_msg)
                    rate.sleep()
                        
                    


            except Exception as e:
                break
        rospy.loginfo("GOAL REACHED")
    else:
        rospy.loginfo("No Path Exists, Check the goal")
        exit(0)


        
if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass


# while not rospy.is_shutdown():
#             print("Path Exists")
#             try:
#                 inter_goal = astar_path.pop(0)
#                 print(inter_goal)
#                 angle = math.atan2((inter_goal[0]-curr_pose[0]),(inter_goal[1]-curr_pose[1]))
#                 # print(inter_goal)

#                 # while(euc_dist(curr_pose,inter_goal)>0.2):
#                 twist_msg = Twist()
                
#                 # sys.stdout.flush()

#                 while(abs(angle-heading)>10*math.pi/180):
#                     print("TURNING:",math.degrees(angle),math.degrees(heading),angle-heading)
#                     if(abs(angle-heading)>90*math.pi/180):
#                         twist_msg.angular.z = 2*(1 if angle > heading else -1)
#                     else:
#                         twist_msg.angular.z = 0.3*(1 if angle > heading else -1)

                    
#                     pub.publish(twist_msg)
#                     rate.sleep()

#                     angle = math.atan2((inter_goal[0]-curr_pose[0]),(inter_goal[1]-curr_pose[1]))
                

#                 twist_msg.angular.z = 0
#                 while(euc_dist(curr_pose,inter_goal)>0.2):


#                     angle = math.atan2((inter_goal[0]-curr_pose[0]),(inter_goal[1]-curr_pose[1]))

#                     print("(X,Y):",(curr_pose[1],curr_pose[0]),"Distance to intermediate:",euc_dist(curr_pose,inter_goal))
#                     # sys.stdout.flush()
#                     if(euc_dist(curr_pose,inter_goal)>0.3):
#                         twist_msg.linear.x = 5
#                     else:
#                         twist_msg.linear.x = euc_dist(curr_pose,inter_goal)

#                     twist_msg.angular.z = abs(angle-heading)*(1 if angle-heading > 0 else -1)
#                     pub.publish(twist_msg)
#                     rate.sleep()

#             except Exception as e:
#                 break
#         rospy.loginfo("GOAL REACHED")