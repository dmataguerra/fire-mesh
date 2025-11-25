/**
 * FireMesh-DX: Sistema Completo (Etapa 2 - Tolerancia a Fallos)
 * Incluye: NTP 4-Timestamps, Buffer Offline (FIFO) y Burst Recovery.
 */

#include <Arduino.h>
#include <painlessMesh.h>
#include <ArduinoJson.h>
#include <deque> // Librería para la Cola FIFO

// ===========================================================================
// 1. CONFIGURACIÓN Y DEFINICIONES
// ===========================================================================

// --- ¡CAMBIAR ESTO SEGÚN LA PLACA QUE ESTÉS GRABANDO! ---
// true = Root Node (Conectado a pared/batería)
// false = Child Node (Conectado a PC para ver monitor)
#define IS_ROOT false

// Credenciales de la Malla
#define   MESH_PREFIX     "FireMesh_DX"
#define   MESH_PASSWORD   "ProyectoFinal123"
#define   MESH_PORT       5555

// Configuración de Memoria Offline
#define MAX_BUFFER_SIZE 20

// ===========================================================================
// 2. ESTRUCTURAS DE DATOS Y OBJETOS
// ===========================================================================

Scheduler userScheduler; 
painlessMesh  mesh;

// Variables de Sincronización
double timeOffset = 0.0;
bool isSynchronized = false;

// Estructura para guardar lecturas cuando no hay red
struct SensorReading {
    uint32_t timestamp; 
    String sensorType;  
    int value;          
};

// El Buffer FIFO (Memoria temporal)
std::deque<SensorReading> offlineBuffer;

// Prototipos de funciones
void sendSyncRequest(); 
void generateSensorData();
void checkAndFlushBuffer();

// Tareas
Task taskSync(10000, TASK_FOREVER, &sendSyncRequest);       // Sincronizar reloj cada 10s
Task taskSensor(5000, TASK_FOREVER, &generateSensorData);   // Leer sensores cada 5s

// ===========================================================================
// 3. LÓGICA DE TIEMPO Y BUFFER (Auxiliares)
// ===========================================================================

// Obtener hora sincronizada (o local ajustada)
unsigned long getNetworkTime() {
  return millis() + (long)timeOffset; 
}

// Función INTELIGENTE de envío: Decide si enviar o guardar
void sendToRoot(SensorReading reading) {
    DynamicJsonDocument doc(1024);
    doc["type"] = "DATA";
    doc["sensor"] = reading.sensorType;
    doc["val"] = reading.value;
    doc["ts"] = reading.timestamp; // TS Original del evento

    String msg;
    serializeJson(doc, msg);

    // Verificamos si tenemos conexión con alguien
    if (mesh.getNodeList().size() == 0) {
        // --- MODO OFFLINE: GUARDAR ---
        Serial.printf("[OFFLINE] Sin red. Guardando %s en buffer FIFO.\n", reading.sensorType.c_str());
        offlineBuffer.push_back(reading);

        // Política FIFO: Borrar el más viejo si se llena
        if (offlineBuffer.size() > MAX_BUFFER_SIZE) {
            offlineBuffer.pop_front();
            Serial.println("[BUFFER FULL] Memoria llena. Borrando dato más antiguo.");
        }
    } else {
        // --- MODO ONLINE: ENVIAR ---
        mesh.sendBroadcast(msg); 
        Serial.printf("[ONLINE] Enviando lectura %s: %d (TS: %u)\n", 
                      reading.sensorType.c_str(), reading.value, reading.timestamp);
        
        // Intentar vaciar pendientes si los hay
        checkAndFlushBuffer();
    }
}

// Mecanismo de Recuperación (Burst Mode)
void checkAndFlushBuffer() {
    if (!offlineBuffer.empty() && mesh.getNodeList().size() > 0) {
        Serial.println("\n>> RECONEXIÓN: Vaciando memoria (Burst)...");
        while (!offlineBuffer.empty()) {
            SensorReading saved = offlineBuffer.front();
            
            DynamicJsonDocument doc(1024);
            doc["type"] = "DATA_RECOVERED"; // Marca especial
            doc["sensor"] = saved.sensorType;
            doc["val"] = saved.value;
            doc["ts"] = saved.timestamp; // Mantiene consistencia histórica
            
            String msg;
            serializeJson(doc, msg);
            mesh.sendBroadcast(msg);
            
            Serial.printf(">> RECUPERADO: %s (TS Original: %u)\n", saved.sensorType.c_str(), saved.timestamp);
            offlineBuffer.pop_front();
            delay(50); // Pequeña pausa para no saturar
        }
        Serial.println(">> Memoria vaciada.\n");
    }
}

// ===========================================================================
// 4. CALLBACKS DE LA MALLA
// ===========================================================================

void receivedCallback( uint32_t from, String &msg ) {
  uint32_t arrivalTimestamp = millis(); // t1 o t3
  DynamicJsonDocument doc(1024);
  deserializeJson(doc, msg);
  String type = doc["type"];

  // --- LÓGICA ROOT (Servidor NTP) ---
  if (IS_ROOT && type == "SYNC_REQ") {
    DynamicJsonDocument res(1024);
    res["type"] = "SYNC_RES";
    res["t0"] = doc["t0"];
    res["t1"] = arrivalTimestamp;
    res["t2"] = millis(); // t2: justo antes de enviar
    
    String resMsg;
    serializeJson(res, resMsg);
    mesh.sendSingle(from, resMsg);
  }

  // --- LÓGICA CHILD (Cliente NTP) ---
  if (!IS_ROOT && type == "SYNC_RES") {
    uint32_t t0 = doc["t0"];
    uint32_t t1 = doc["t1"];
    uint32_t t2 = doc["t2"];
    uint32_t t3 = arrivalTimestamp;

    // Fórmula NTP 4-Timestamps
    double offset = ((double)(t1 - t0) + (double)(t2 - t3)) / 2.0;
    timeOffset = offset;
    isSynchronized = true;
    Serial.printf("[NTP] Sincronizado. Offset: %.2f ms\n", timeOffset);
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> Conexión establecida: %u\n", nodeId);
  checkAndFlushBuffer(); // Intentar vaciar buffer al conectar
}

// ===========================================================================
// 5. TAREAS
// ===========================================================================

void sendSyncRequest() {
  if (!IS_ROOT && mesh.getNodeList().size() > 0) {
    DynamicJsonDocument doc(1024);
    doc["type"] = "SYNC_REQ";
    doc["t0"] = millis();
    String msg;
    serializeJson(doc, msg);
    mesh.sendBroadcast(msg);
  }
}

void generateSensorData() {
  if (!IS_ROOT) {
    SensorReading lectura;
    lectura.timestamp = getNetworkTime();
    
    // Simular Sensores
    if (random(0, 100) > 50) {
        lectura.sensorType = "HUMO";
        lectura.value = random(300, 900);
    } else {
        lectura.sensorType = "FLAMA";
        lectura.value = random(0, 1);
    }
    sendToRoot(lectura);
  }
}

// ===========================================================================
// 6. SETUP & LOOP
// ===========================================================================

void setup() {
  Serial.begin(115200);
  
  // Inicializar Mesh
  mesh.setDebugMsgTypes( ERROR | STARTUP ); 
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);

  // Activar Tareas solo si soy Child
  if (!IS_ROOT) {
    userScheduler.addTask(taskSync);
    taskSync.enable();
    userScheduler.addTask(taskSensor);
    taskSensor.enable();
    Serial.println("=== CHILD NODE INICIADO ===");
  } else {
    Serial.println("=== ROOT NODE INICIADO ===");
  }
}

void loop() {
  mesh.update();
}
