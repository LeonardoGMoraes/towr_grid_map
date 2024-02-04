import rosbag
import numpy as np

ee_contact = []

x_com_x = []
x_com_y = []
x_com_z = []

xd_com_x  = []
xd_com_y  = []
xd_com_z  = []

x_ori_x = []
x_ori_y = []
x_ori_z = []
x_ori_w = []

x_angvel_x = []
x_angvel_y = []
x_angvel_z = []

x_des_1_x = []
x_des_1_y = []
x_des_1_z = []

x_des_2_x = []
x_des_2_y = []
x_des_2_z = []

x_des_3_x = []
x_des_3_y = []
x_des_3_z = []

x_des_4_x = []
x_des_4_y = []
x_des_4_z = []

xd_des_1_x=[]
xd_des_1_y=[]
xd_des_1_z=[]

xd_des_2_x=[]
xd_des_2_y=[]
xd_des_2_z=[]

xd_des_3_x=[]
xd_des_3_y=[]
xd_des_3_z=[]

xd_des_4_x=[]
xd_des_4_y=[]
xd_des_4_z=[]



bag = rosbag.Bag('/home/leo/.ros/towr_trajectory2.bag')
for topic, msg, t in bag.read_messages (topics=['/xpp/state_des']):
    print(msg)

    #Vetor de contato dos pes
    ee_contact.append(msg.ee_contact)

    x_des_1_x.append(msg.ee_motion[0].pos.x)
    x_des_1_y.append(msg.ee_motion[0].pos.y)
    x_des_1_z.append(msg.ee_motion[0].pos.z)

    x_des_2_x.append(msg.ee_motion[1].pos.x)
    x_des_2_y.append(msg.ee_motion[1].pos.y)
    x_des_2_z.append(msg.ee_motion[1].pos.z)

    x_des_3_x.append(msg.ee_motion[2].pos.x)
    x_des_3_y.append(msg.ee_motion[2].pos.y)
    x_des_3_z.append(msg.ee_motion[2].pos.z)

    x_des_4_x.append(msg.ee_motion[3].pos.x)
    x_des_4_y.append(msg.ee_motion[3].pos.y)
    x_des_4_z.append(msg.ee_motion[3].pos.z)

    xd_des_1_x.append(msg.ee_motion[0].vel.x)
    xd_des_1_y.append(msg.ee_motion[0].vel.y)
    xd_des_1_z.append(msg.ee_motion[0].vel.z)

    xd_des_2_x.append(msg.ee_motion[1].vel.x)
    xd_des_2_y.append(msg.ee_motion[1].vel.y)
    xd_des_2_z.append(msg.ee_motion[1].vel.z)

    xd_des_3_x.append(msg.ee_motion[2].vel.x)
    xd_des_3_y.append(msg.ee_motion[2].vel.y)
    xd_des_3_z.append(msg.ee_motion[2].vel.z)

    xd_des_4_x.append(msg.ee_motion[3].vel.x)
    xd_des_4_y.append(msg.ee_motion[3].vel.y)
    xd_des_4_z.append(msg.ee_motion[3].vel.z)

    x_com_x.append(msg.base.pose.position.x)
    x_com_y.append(msg.base.pose.position.y)
    x_com_z.append(msg.base.pose.position.z)

    x_ori_x.append(msg.base.pose.orientation.x)
    x_ori_y.append(msg.base.pose.orientation.y)
    x_ori_z.append(msg.base.pose.orientation.z)
    x_ori_w.append(msg.base.pose.orientation.w)

    xd_com_x.append(msg.base.twist.linear.x)
    xd_com_y.append(msg.base.twist.linear.y)
    xd_com_z.append(msg.base.twist.linear.z)

    x_angvel_x.append(msg.base.twist.angular.x)
    x_angvel_y.append(msg.base.twist.angular.y)
    x_angvel_z.append(msg.base.twist.angular.z)

    #print(msg.ee_motion[3].vel.z)
    #input('teste')
bag.close()

#print (x_angvel_z)