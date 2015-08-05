------------------------HOW TO RUN ROSLAUNCH------------------------

While roscore/ros_stage is running, open another terminal tab and cd into ugbots_ros

just type command
"roslaunch ugbots_ros example.launch"

replace 'example' with which ever launch file you want to run



-----------------UNDERSTANDING THE SIMPLE EXAMPLE-------------------

#We are using roslaunch to run different robots of the same type(node)
#This is just an example and you'll probably need to build your own launch file later

<launch>
	#you select the namespace, by default the first robot made in the world is robot_0
	<group ns="robot_0">
		#pkg is the package file name obviously
		#not sure what name is for, but it can be anything
		#the type is what the robot node is called, this is initialised in the CMakeLists and your cpp file
		<node pkg="ugbots_ros" name="robotnode" type="R0"/>
	</group>
	#Now I'm selecting the second robot and using the same robot node, but without a conflict 
	<group ns="robot_1">
		<node pkg="ugbots_ros" name="robotnode" type="R0"/>
	</group>
	#The next two show that they're using R1.cpp at the same time
	#You can see in the world that 2 robots are doing the same thing, while the other 2 do something else the same
	<group ns="robot_2">
		<node pkg="ugbots_ros" name="robotnode" type="R1"/>
	</group>
	<group ns="robot_3">
		<node pkg="ugbots_ros" name="robotnode" type="R1"/>
	</group>
	#If you don't know what happens when you try to run the same node, try now
</launch>
