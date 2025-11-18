/*
	task_firebase_upload.cpp — Tarea de subida ordenada a Firebase (FIFO).

	- Consume la cola de mensajes pendientes.
	- Empuja datos a Firebase en orden (First-In, First-Out).
	- Implementa reintentos ante fallos de internet.
*/

