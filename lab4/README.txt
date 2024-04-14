Il seguente workspace è formato da tre distinti package
  - py_srvcli                    (per il codice del server e client del servizio)
  - tutorial_interface           (per la definizione del servizio AddThreeInts) 
  - my_package			 (per interagire con turtlesim)


ros2 run py_srvcli service		(in un terminale) (serve a far partire il server del servizio)
ros2 run py_srvcli client 2 3 5		(nell'altro terminale) (serve a richiedere col client un servizio)


Per far ruotare la tartaruga in base alla richiesta esplicita di un servizio: (OGNI COMANDO VA FATTO PARTIRE IN UN TERMINALE DIVERSO)
 ros2 run turtlesim turtlesim_node		(per far partire turtlesin)
 ros2 run my_package turtle_service_server	(per far partire il server del servizio) 
						(tale server aspetta le richeste del client per poter interagire con la tartaruga)
<<<<<<< HEAD
 ros2 run my_package turtle_service_client 1	(per poter richiedere il servizio TurnTurtle e dunque far ruotare la tartaruga)
=======
 ros2 run my_package turtle_service_client 1	(per poter richiedere il servizio TurnTurtle e dunque far ruotare la tartaruga)
>>>>>>> 27d3f1c4176713f917dcf64a654e0783d5573753
