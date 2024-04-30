Il seguente workspace è per far comunciare ros con gazebo.
Nel file "multicopter.sdf" è definito un singolo drone nominato "X3". Tale
file viene aperto su Gazebo attraverso l'apposito launch file "launch_gazebo.py".
Tale file definisce anche due bridge per due topic in cui si scrive a si 
ascolta:
	- BRIDGE 1:	cmd_vel		(qui si scrive)
	- BRIDGE 2:	odometry	(qui si ascolta)
L'idea è che nel topic "cmd_vel" diamo ordini di movimento al drone. Nel topic
"odometry" otteniamo lo stato del drone (in termini di coordinate e altro) in 
modo aggiornato. 

Nel file "my_node2.py" viene definiti gli ordini da dare al drone. In tale
file indichiamo l'altezza da raggiungere e un determinato punto del piano
verso cui il drone si deve voltare. 


Per testare il tutto, dare il seguente comando:
colcon build; source ./install/local_setup.bash; ros2 launch launch_gazebo.py



