from panda_py import libfranka

HOST = "172.16.0.2"


width_1 = 0.0
width_2 = 0.08

gripper = libfranka.Gripper(HOST)

for i in range(7):
    gripper.move(width_1, speed = 2.0)
    width_1 = width_1 + 0.01

# exit()

gripper.move(width_1, speed = 1.0)
# gripper.grasp(width, speed, force, epsilon_inner, epsilon_outer)
