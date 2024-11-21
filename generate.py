import pyrosim.pyrosim as ps

# Global parameters
l = 1 # length
w = 1 # width 
h = 1 # height 

# x = 0
# y = 0
# z = 0.5
x = 0
y = 0
z = 1.5

# def Create_World():
#     ps.Start_SDF("box.sdf")
#     for i in range(10):
#         ps.Send_Cube(name="Box",pos=[x,y,z],size=[l,w,h])
#         z += h
#         l = 0.9 * l
#         w = 0.9 * w
#         h = 0.9 * h
#     ps.End()

def Create_Robot1():
    ps.Start_URDF("body.urdf")
    ps.Send_Cube(name="Foot",pos=[x,y,z],size=[l,w,h]) # Parent
    ps.Send_Joint(name="Foot_Torso", parent="Foot", child="Torso", type="revolute", position = [0.4,0,1.0])
    ps.Send_Cube(name="Torso",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child
    ps.Send_Joint(name="Foot_Torso2", parent="Foot", child="Torso2", type="revolute", position = [-0.4,0.0,1.0])
    ps.Send_Cube(name="Torso2",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child


    ps.Send_Joint(name="Foot_Torso3", parent="Foot", child="Torso3", type="revolute", position = [0.4,-0.8,1.0])
    ps.Send_Cube(name="Torso3",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child
    ps.Send_Joint(name="Foot_Torso4", parent="Foot", child="Torso4", type="revolute", position = [-0.4,-0.8,1.0])
    ps.Send_Cube(name="Torso4",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="Torso_knee4", parent="Torso4", child="knee4", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="knee4",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="Torso_knee3", parent="Torso3", child="knee3", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="knee3",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="Torso_knee2", parent="Torso2", child="knee2", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="knee2",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="Torso_knee1", parent="Torso", child="knee1", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="knee1",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="knee_ankle4", parent="knee4", child="ankle4", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="ankle4",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="knee_ankle3", parent="knee3", child="ankle3", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="ankle3",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="knee_ankle2", parent="knee2", child="ankle2", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="ankle2",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="knee_ankle1", parent="knee1", child="ankle1", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="ankle1",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child


    ps.Send_Joint(name="ankle_foot4", parent="ankle4", child="foot4", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="foot4",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="ankle_foot3", parent="ankle3", child="foot3", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="foot3",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="ankle_foot2", parent="ankle2", child="foot2", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="foot2",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="ankle_foot1", parent="ankle1", child="foot1", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="foot1",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child


    ps.Send_Joint(name="foot_toe4", parent="foot4", child="toe4", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="toe4",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="foot_toe3", parent="foot3", child="toe3", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="toe3",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="foot_toe2", parent="foot2", child="toe2", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="toe2",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="foot_toe1", parent="foot1", child="toe1", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="toe1",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child


    ps.Send_Joint(name="toe_toenail4", parent="toe4", child="toenail4", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="toenail4",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="toe_toenail3", parent="toe3", child="toenail3", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="toenail3",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="toe_toenail2", parent="toe2", child="toenail2", type="revolute", position = [-0.2,-0.0,-0.0])
    ps.Send_Cube(name="toenail2",pos=[-0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child

    ps.Send_Joint(name="toe_toenail1", parent="toe1", child="toenail1", type="revolute", position = [0.2,-0.0,-0.0])
    ps.Send_Cube(name="toenail1",pos=[0.1,0.4,0.1],size=[0.2,0.2,0.2]) # Child  

    ps.Send_Sensor_Neuron(name=f"Touch_Sensor_1", linkName="toenail1")
    ps.Send_Sensor_Neuron(name=f"Touch_Sensor_2", linkName="toenail2")
    ps.Send_Sensor_Neuron(name=f"Touch_Sensor_3", linkName="toenail3")
    ps.Send_Sensor_Neuron(name=f"Touch_Sensor_4", linkName="toenail4")


    
    
    ps.End()

def Create_Robot2():

    ps.Start_URDF("body2.urdf")
    ps.Send_Cube(name="parent",pos=[2,2,1.5],size=[l,w,h]) # Parent
    
    ps.Send_Joint(name="parent_child2", parent="parent", child="child2", type="revolute", position = [1.5,1.2,1.0])
    ps.Send_Cube(name="child2",pos=[-0.6,0.4,0.1],size=[1.2,0.2,0.2]) # Child


    ps.Send_Joint(name="parent_child1", parent="parent", child="child1", type="revolute", position = [2.5,1.4,1.0])
    ps.Send_Cube(name="child1",pos=[0.6,0.4,0.1],size=[1.2,0.2,0.2]) # Child
    
    # ps.Send_Joint(name="parent_child4", parent="parent", child="child4", type="revolute", position = [1.5,1.4,1.0])
    # ps.Send_Cube(name="child4",pos=[-0.6,0.4,0.1],size=[1.2,0.2,0.2]) # Child


    ps.End()

Create_Robot1()
Create_Robot2()