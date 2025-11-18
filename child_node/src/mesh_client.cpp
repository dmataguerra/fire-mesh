/*
	mesh_client.cpp — Cliente de malla para el Child Node.

	- Conecta y envía mensajes al Root Node.
	- Soporta broadcast y envíos en ráfaga (cuando hay FIFO pendiente).
	- Detecta desconexión mediante heartbeats.
*/

