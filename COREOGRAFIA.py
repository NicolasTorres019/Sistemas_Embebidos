# -*- coding: utf-8 -*-
from machine import Pin, PWM, RTC, I2C
import network, socket, time, ubinascii, uhashlib
import ujson
import _thread
import errno
import math
import ntptime
from imu import MPU6050 # ¡IMPORTANTE! Asegúrate de tener el archivo imu.py

# ============================================================
# 0. CONFIGURACIÓN DEL BRÓKER Y ROBOT
# ============================================================
BROKER_IP = "192.168.1.101" # <--- ¡PON AQUÍ LA IP REAL DE TU PC!
BROKER_PORT = 5051

# --- Constantes Físicas del Robot (¡AJUSTAR!) ---
DISTANCIA_ENTRE_RUEDAS = 0.135
# VELOCIDAD_MAXIMA_ROBOT = 0.5 # Ya no es necesaria para escalar v

# ### NUEVO ### Factor para escalar la velocidad angular w (0-10) a un rango mayor
W_SCALING_FACTOR = 30.0 # Ajusta este valor si es necesario (e.g., 15.0, 20.0)

broker_socket = None
broker_lock = _thread.allocate_lock()

# Almacenes para secuencias y tareas programadas
secuencias_guardadas = {}
tareas_programadas = []

# ============================================================
# 0.1 CONFIGURACIÓN DE LÍMITES DE SERVOS
# ============================================================
SERVO1_MIN_ANGLE = 0; SERVO1_MAX_ANGLE = 90
SERVO2_MIN_ANGLE = 30; SERVO2_MAX_ANGLE = 180
SERVO3_MIN_ANGLE = 10; SERVO3_MAX_ANGLE = 180

# ============================================================
# 1. CONFIGURACIÓN DE HARDWARE
# ============================================================
# --- Servos ---
servo1 = PWM(Pin(10)); servo2 = PWM(Pin(11)); servo3 = PWM(Pin(12))
for s in [servo1, servo2, servo3]: s.freq(50)
angulo_servo1 = 90; angulo_servo2 = 90; angulo_servo3 = 115

# --- Motores (Puente H) ---
IN1 = Pin(3, Pin.OUT); IN2 = Pin(4, Pin.OUT)
ENA = PWM(Pin(2)); ENA.freq(1000)
IN3 = Pin(21, Pin.OUT); IN4 = Pin(20, Pin.OUT)
ENB = PWM(Pin(19)); ENB.freq(1000)
MAX_PWM = 65535
MIN_PWM = int(MAX_PWM * 0.45)

# --- Sensor MPU6050 ---
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400000)
imu = MPU6050(i2c)

# --- Parámetros de Calibración y Control PID ---
CALIBRATION_SAMPLES = 200
ay_offset, gz_offset = 0.0, 0.0
ACCEL_THRESHOLD = 0.04
VEL_FILT_ALPHA = 0.9
KP_VEL = 2800.0; KI_VEL = 800.0; KD_VEL = 0.0
KP_YAW = 750.0; KI_YAW = 900.0; KD_YAW = 0.0
BOOST_PWM = 42000
BOOST_DURATION_S = 0.25

# ============================================================
# 2. FUNCIONES DE MOVIMIENTO
# ============================================================
def mover_servo(servo, angulo):
    global angulo_servo1, angulo_servo2, angulo_servo3
    if servo == servo1: angulo = max(SERVO1_MIN_ANGLE, min(SERVO1_MAX_ANGLE, angulo)); angulo_servo1 = angulo
    elif servo == servo2: angulo = max(SERVO2_MIN_ANGLE, min(SERVO2_MAX_ANGLE, angulo)); angulo_servo2 = angulo
    elif servo == servo3: angulo = max(SERVO3_MIN_ANGLE, min(SERVO3_MAX_ANGLE, angulo)); angulo_servo3 = angulo
    min_us = 500; max_us = 2500
    us = min_us + (angulo / 180) * (max_us - min_us)
    duty = int(us * 65535 / 20000)
    servo.duty_u16(duty)

def mover_servo_tiempo(angulo_final1, angulo_final2, angulo_final3, tiempo_total):
    global angulo_servo1, angulo_servo2, angulo_servo3; pasos = 50
    if tiempo_total <= 0:
        mover_servo(servo1, angulo_final1); mover_servo(servo2, angulo_final2); mover_servo(servo3, angulo_final3)
        return
    delay = tiempo_total / pasos; angulo_inicial1, angulo_inicial2, angulo_inicial3 = angulo_servo1, angulo_servo2, angulo_servo3
    for i in range(pasos + 1):
        ang1 = angulo_inicial1 + (angulo_final1 - angulo_inicial1) * i / pasos
        ang2 = angulo_inicial2 + (angulo_final2 - angulo_inicial2) * i / pasos
        ang3 = angulo_inicial3 + (angulo_final3 - angulo_inicial3) * i / pasos
        mover_servo(servo1, ang1); mover_servo(servo2, ang2); mover_servo(servo3, ang3); time.sleep(delay)
    angulo_servo1, angulo_servo2, angulo_servo3 = angulo_final1, angulo_final2, angulo_final3

def controlar_motores_pid(pwm_izq_input, pwm_der_input):
    if pwm_izq_input == 0:
        ENA.duty_u16(0); IN1.value(0); IN2.value(0)
    else:
        pwm_abs_izq = abs(pwm_izq_input)
        pwm_duty_izq = MIN_PWM if pwm_abs_izq < MIN_PWM else pwm_abs_izq
        ENA.duty_u16(int(min(pwm_duty_izq, MAX_PWM)))
        if pwm_izq_input > 0: IN1.value(1); IN2.value(0)
        else: IN1.value(0); IN2.value(1)
    if pwm_der_input == 0:
        ENB.duty_u16(0); IN3.value(0); IN4.value(0)
    else:
        pwm_abs_der = abs(pwm_der_input)
        pwm_duty_der = MIN_PWM if pwm_abs_der < MIN_PWM else pwm_abs_der
        ENB.duty_u16(int(min(pwm_duty_der, MAX_PWM)))
        if pwm_der_input > 0: IN3.value(1); IN4.value(0)
        else: IN3.value(0); IN4.value(1)

def detener_motores():
    controlar_motores_pid(0, 0)
    
def mover_robot_simple(vel_izq, vel_der):
    if vel_izq > 0: IN1.value(0); IN2.value(1)
    elif vel_izq < 0: IN1.value(1); IN2.value(0)
    else: IN1.value(0); IN2.value(0)
    if vel_der > 0: IN3.value(0); IN4.value(1)
    elif vel_der < 0: IN3.value(1); IN4.value(0)
    else: IN3.value(0); IN4.value(0)
    ENA.duty_u16(int(abs(vel_izq) / 100 * 65535)); ENB.duty_u16(int(abs(vel_der) / 100 * 65535))

# ============================================================
# 3. EJECUTORES DE COMANDOS
# ============================================================
def calibrar_sensores():
    global ay_offset, gz_offset
    print("\nCalibrando sensores... Mantén el robot quieto.")
    sum_ay, sum_gz = 0.0, 0.0
    for _ in range(CALIBRATION_SAMPLES):
        sum_ay += (imu.accel.y * -1)
        sum_gz += (imu.gyro.z * -1)
        time.sleep(0.01)
    ay_offset = sum_ay / CALIBRATION_SAMPLES
    gz_offset = sum_gz / CALIBRATION_SAMPLES
    print(f"Calibración completa. Offsets: ay={ay_offset:.4f}, gz={gz_offset:.4f}")

def mover_a_velocidad(duracion, velocidad_deseada_dms, velocidad_angular_dps=0.0, servo_targets=None):
    print(f"\nIniciando movimiento PID: {velocidad_deseada_dms} dm/s, {velocidad_angular_dps} dps, por {duracion}s")
    start_time_ms = time.ticks_ms(); last_time_us = time.ticks_us()
    vy_filt = 0.0; integral_vel, last_error_vel = 0.0, 0.0
    integral_yaw, last_error_yaw = 0.0, 0.0
    s_inicial1, s_inicial2, s_inicial3 = angulo_servo1, angulo_servo2, angulo_servo3
    s_final1, s_final2, s_final3 = servo_targets if servo_targets else (s_inicial1, s_inicial2, s_inicial3)

    while time.ticks_diff(time.ticks_ms(), start_time_ms) < duracion * 1000:
        try:
            current_time_us = time.ticks_us()
            dt = time.ticks_diff(current_time_us, last_time_us) / 1_000_000.0
            last_time_us = current_time_us
            if dt <= 0: continue
            if duracion > 0:
                progreso = min(1.0, time.ticks_diff(time.ticks_ms(), start_time_ms) / (duracion * 1000))
                ang1 = s_inicial1 + (s_final1 - s_inicial1) * progreso
                ang2 = s_inicial2 + (s_final2 - s_inicial2) * progreso
                ang3 = s_inicial3 + (s_final3 - s_inicial3) * progreso
                mover_servo(servo1, ang1); mover_servo(servo2, ang2); mover_servo(servo3, ang3)
            ay_raw = (imu.accel.y * -1) - ay_offset; gz_raw = (imu.gyro.z * -1) - gz_offset
            ay = ay_raw if abs(ay_raw) >= ACCEL_THRESHOLD else 0.0
            gz = gz_raw if abs(gz_raw) >= 1.5 else 0.0
            if velocidad_deseada_dms != 0:
                vy_filt = VEL_FILT_ALPHA * (vy_filt + ay * 98.1 * dt) + (1 - VEL_FILT_ALPHA) * vy_filt
            else:
                vy_filt = 0.0
            potencia_base = 0.0
            if velocidad_deseada_dms != 0:
                if time.ticks_diff(time.ticks_ms(), start_time_ms) / 1000 < BOOST_DURATION_S:
                    potencia_base = BOOST_PWM if velocidad_deseada_dms >= 0 else -BOOST_PWM; integral_vel = 0
                else:
                    error_vel = velocidad_deseada_dms - vy_filt; integral_vel = max(-1.0, min(1.0, integral_vel + error_vel * dt))
                    derivative_vel = (error_vel - last_error_vel) / dt if dt > 0 else 0; last_error_vel = error_vel
                    potencia_base = (KP_VEL * error_vel) + (KI_VEL * integral_vel) + (KD_VEL * derivative_vel)
                    limite = MAX_PWM * 0.9; potencia_base = max(-limite, min(potencia_base, limite))
            correccion = 0.0
            if velocidad_angular_dps != 0:
                error_yaw = velocidad_angular_dps - gz; integral_yaw = max(-1.0, min(1.0, integral_yaw + error_yaw * dt))
                derivative_yaw = (error_yaw - last_error_yaw) / dt if dt > 0 else 0; last_error_yaw = error_yaw
                correccion = (KP_YAW * error_yaw) + (KI_YAW * integral_yaw) + (KD_YAW * derivative_yaw)
            else:
                integral_yaw = 0.0; last_error_yaw = 0.0
            pwm_izq = potencia_base + correccion; pwm_der = potencia_base - correccion
            controlar_motores_pid(pwm_izq, pwm_der)
            print(f"Vy={vy_filt:6.2f} | Gz_Obj={velocidad_angular_dps:4.1f} | Gz={gz:6.2f} | PWM_B={int(potencia_base):6d} | Corr={int(correccion):4d}", end='\r')
        except Exception as e:
            print(f"\nError en bucle PID: {e}"); pass
    detener_motores()
    print(f"\nMovimiento PID finalizado.")

def ejecutar_secuencia_json(secuencia):
    if not isinstance(secuencia, dict) or "states" not in secuencia or not isinstance(secuencia["states"], list):
        print("ERROR: Formato de secuencia JSON incorrecto."); return
    nombre_secuencia = secuencia.get("name", "sin_nombre")
    print(f">>> INICIANDO SECUENCIA '{nombre_secuencia}'")
    for paso in secuencia["states"]:
        print(f"  - Ejecutando paso: {paso}")
        ejecutar_estado(paso)
    print(f">>> SECUENCIA '{nombre_secuencia}' TERMINADA")

def ejecutar_estado(estado):
    print(">>> INICIANDO COMANDO DE ESTADO")
    v_dms = estado.get("v", 0.0) # Leemos directamente como dm/s
    w_dps_input = estado.get("w", 0.0) # Leemos w (0-10)
    t = estado.get("duration", 0.0)
    v_dms = v_dms*60
    # ### CAMBIO CLAVE ###: Escalar w_dps_input
    w_dps_scaled = w_dps_input * W_SCALING_FACTOR
    print(f"  - Velocidad angular recibida: {w_dps_input}, escalada a: {w_dps_scaled}")

    logical_s1 = estado.get("alfa2", angulo_servo1); logical_s2 = estado.get("alfa1", angulo_servo2)
    logical_s3 = estado.get("alfa0", angulo_servo3)
    physical_s1 =  logical_s1+90; physical_s2 = 180-logical_s2; physical_s3 = logical_s3 + 95
    print(f"  - Mapeo: (alfa0:{logical_s1}->s1:{physical_s1}), (alfa2:{logical_s2}->s2:{physical_s2}), (alfa1:{logical_s3}->s3:{physical_s3})")
    
    # Pasamos el w escalado a la función de movimiento
    mover_a_velocidad(duracion=t, velocidad_deseada_dms=v_dms, velocidad_angular_dps=w_dps_scaled, servo_targets=(physical_s1, physical_s2, physical_s3))
    print(">>> COMANDO DE ESTADO TERMINADO")

# ============================================================
# 4. CONFIGURACIÓN WIFI Y HORA
# ============================================================
SSID = "Ejemplo"; PASSWORD = "12345678"
wlan = network.WLAN(network.STA_IF); wlan.active(True); wlan.connect(SSID, PASSWORD)
print("Conectando a la red WiFi...");
connect_timeout = time.time() + 15
while not wlan.isconnected() and time.time() < connect_timeout:
    print(".")
    time.sleep(1)

if wlan.isconnected():
    print("Conexión exitosa. IP:", wlan.ifconfig()[0])
    print("Sincronizando reloj desde internet (NTP)...")
    NTP_SYNCHRONIZED = False; NTP_RETRIES = 5
    ntp_server_host = "pool.ntp.org"
    for i in range(NTP_RETRIES):
        try:
            print(f"Intento NTP {i+1}/{NTP_RETRIES} con {ntp_server_host}...")
            ntptime.host = ntp_server_host; ntptime.settime()
            UTC_OFFSET = -5 * 3600
            hora_local = time.time() + UTC_OFFSET; tm = time.localtime(hora_local)
            RTC().datetime((tm[0], tm[1], tm[2], tm[6], tm[3], tm[4], tm[5], 0))
            NTP_SYNCHRONIZED = True
            print(f"¡Reloj sincronizado a la hora de Bogotá!: {time.localtime()[3]:02d}:{time.localtime()[4]:02d}:{time.localtime()[5]:02d}")
            break
        except Exception as e:
            print(f"ERROR en intento NTP {i+1}: {e}")
            if i < NTP_RETRIES - 1: print("Reintentando en 5 segundos..."); time.sleep(5)
    if not NTP_SYNCHRONIZED: print("ADVERTENCIA: No se pudo sincronizar la hora con NTP.")
    calibrar_sensores()
else:
    print("!!! ERROR FATAL: No se pudo conectar a la red WiFi.")

# ============================================================
# 5. INTERFAZ WEB
# ============================================================
HTML_PAGE = """
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Panel de Control del Robot</title>
<style>
    :root {
        --bg-color: #1a1a2e; --primary-color: #16213e; --secondary-color: #0f3460;
        --accent-color: #e94560; --font-color: #dcdcdc; --shadow-color: rgba(0,0,0,0.4);
    }
    body {
        font-family: 'Segoe UI', sans-serif; background-color: var(--bg-color);
        color: var(--font-color); display: flex; justify-content: center;
        align-items: center; min-height: 100vh; margin: 0; padding: 10px; box-sizing: border-box;
    }
    .container {
        background-color: var(--primary-color); padding: 25px; border-radius: 20px;
        box-shadow: 0 10px 30px var(--shadow-color); width: 100%; max-width: 420px; text-align: center;
    }
    h1 { color: var(--accent-color); margin-top: 0; }
    h2 {
        color: var(--accent-color); margin-top: 30px; margin-bottom: 20px;
        border-bottom: 2px solid var(--secondary-color); padding-bottom: 10px;
    }
    .d-pad-container {
        display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: 1fr 1fr 1fr;
        gap: 10px; width: 180px; height: 180px; margin: 20px auto;
        grid-template-areas: ". up ." "left . right" ". down .";
    }
    .d-pad-btn {
        background-color: var(--secondary-color); border: none; color: var(--font-color);
        font-size: 2.5em; cursor: pointer; border-radius: 15px; transition: background-color 0.2s;
        display: flex; justify-content: center; align-items: center;
        user-select: none;
    }
    .d-pad-btn:hover, .d-pad-btn:active { background-color: #e94560a0; }
    #btn-up { grid-area: up; } #btn-down { grid-area: down; }
    #btn-left { grid-area: left; } #btn-right { grid-area: right; }
    .servo-controls { display: flex; flex-direction: column; gap: 20px; }
    .servo { display: flex; flex-direction: column; align-items: center; gap: 8px; }
    .servo label { font-size: 1.1em; }
    input[type=range] { width: 90%; }
    .broker-controls { margin-top: 20px; }
    .broker-controls input[type=text] {
        width: 100%; padding: 12px; margin-bottom: 10px; border-radius: 8px;
        border: 1px solid var(--secondary-color); background-color: var(--bg-color);
        color: white; box-sizing: border-box; font-size: 1em;
    }
    .broker-controls button {
        width: 100%; padding: 12px; background-color: var(--secondary-color);
        color: white; border: none; border-radius: 8px; cursor: pointer;
        font-size: 1.1em; transition: background-color 0.2s;
    }
    .broker-controls button:hover, .broker-controls button:active { background-color: #1a4f8a; }
</style>
</head>
<body>
<div class="container">
    <h1>🤖 Control Robot</h1>
    <h2>Movimiento</h2>
    <div class="d-pad-container">
        <button id="btn-up" class="d-pad-btn">↑</button>
        <button id="btn-left" class="d-pad-btn">←</button>
        <button id="btn-right" class="d-pad-btn">→</button>
        <button id="btn-down" class="d-pad-btn">↓</button>
    </div>
    <h2>Servos</h2>
    <div class="servo-controls">
        <div class="servo"><label>Servo 1: <span id="servo1_output">90°</span></label><input type="range" id="servo1" min="0" max="180" value="90"></div>
        <div class="servo"><label>Servo 2: <span id="servo2_output">90°</span></label><input type="range" id="servo2" min="0" max="180" value="90"></div>
        <div class="servo"><label>Servo 3: <span id="servo3_output">115°</span></label><input type="range" id="servo3" min="0" max="180" value="115"></div>
    </div>
    <h2>Bróker</h2>
    <div class="broker-controls">
      <input type="text" id="topic-input" placeholder="Escribe el tópico aquí..." value="UDFJC/emb1/robot0/RPi/command">
      <button id="btn-subscribe">Suscribir</button>
    </div>
</div>
<script>
document.addEventListener('DOMContentLoaded',()=>{
 const ws=new WebSocket("ws://"+location.host+"/");
 const send=(c)=>{if(ws.readyState===WebSocket.OPEN)ws.send(c);}
 const moves={'btn-up':'forward','btn-down':'backward','btn-left':'left','btn-right':'right'};
 for(const[id,cmd]of Object.entries(moves)){
  const b=document.getElementById(id);
  const start=(e)=>{ e.preventDefault(); send(cmd+'_start'); }
  const stop=(e)=>{ e.preventDefault(); send(cmd+'_stop'); }
  b.addEventListener('mousedown',start);
  b.addEventListener('mouseup',stop);
  b.addEventListener('mouseleave',stop);
  b.addEventListener('touchstart',start, {passive: false});
  b.addEventListener('touchend',stop);
 }
 for(let i=1;i<=3;i++){
  const s=document.getElementById('servo'+i);
  const o=document.getElementById('servo'+i+'_output');
  s.addEventListener('input',()=>{o.textContent=s.value+'°';send('servo'+i+'_'+s.value);});
 }
 document.getElementById('btn-subscribe').addEventListener('click', () => {
   const topic = document.getElementById('topic-input').value.trim();
   if (topic) {
     send('subscribe_' + topic);
     alert('Solicitud de suscripción enviada para: ' + topic);
   } else {
     alert('Por favor, escribe un tópico.');
   }
 });
});
</script>
</body></html>
"""

# ============================================================
# 6. SERVIDOR WEBSOCKET
# ============================================================
def websocket_handshake(client, request):
    key = None;
    for line in request.split(b"\r\n"):
        if line.startswith(b"Sec-WebSocket-Key:"): key = line.split(b":", 1)[1].strip(); break
    if not key: return False
    accept_key = ubinascii.b2a_base64(uhashlib.sha1(key + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest()).decode().strip()
    client.send(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " + accept_key.encode() + b"\r\n\r\n")
    return True
def recv_ws_frame(client):
    try:
        hdr = client.recv(2);
        if not hdr or len(hdr) < 2: return None
        length = hdr[1] & 0x7F; mask_key = client.recv(4); data = bytearray(client.recv(length))
        for i in range(length): data[i] ^= mask_key[i % 4]
        return data.decode()
    except: return None

# ============================================================
# 7. LÓGICA DEL CLIENTE DEL BRÓKER
# ============================================================
def subscribe_to_topic(topic):
    global broker_socket
    with broker_lock:
        if broker_socket:
            try:
                pkt = {"action": "SUB", "topic": topic}; payload = ujson.dumps(pkt) + "\n"
                broker_socket.send(payload.encode('utf-8')); print(f"-> Enviada suscripción: {topic}")
            except Exception as e: print(f"Error al suscribir: {e}"); broker_socket = None

def broker_listener_thread():
    global broker_socket, secuencias_guardadas, tareas_programadas
    buffer = b""
    while True:
        # Bloque para revisar la agenda de tareas
        ahora_timestamp = time.time()
        tareas_para_ejecutar = [t for t in tareas_programadas if ahora_timestamp >= t['trigger_timestamp']]
        for tarea in tareas_para_ejecutar:
            print(f"  - ¡HORA DE EJECUTAR TAREA PROGRAMADA! Nombre: {tarea['name']}")
            if tarea['name'] in secuencias_guardadas: ejecutar_secuencia_json(secuencias_guardadas[tarea['name']])
            else: print(f"  - ERROR: La secuencia programada '{tarea['name']}' ya no existe.")
            tareas_programadas.remove(tarea)

        # Bloque para conectar/reconectar al bróker
        if not broker_socket:
            try:
                addr_info = socket.getaddrinfo(BROKER_IP, BROKER_PORT)[0][-1]
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM); s.settimeout(1.0)
                s.connect(addr_info)
                with broker_lock: broker_socket = s
                print("¡ÉXITO! Conectado al bróker.")
            except Exception as e: print(f"!!! FALLÓ LA CONEXIÓN: {e}"); broker_socket = None; time.sleep(5); continue
        
        # Bloque para escuchar mensajes del bróker
        try:
            broker_socket.settimeout(0.1)
            data = broker_socket.recv(1024)
            if not data:
                print("Bróker cerró la conexión.");
                with broker_lock:
                    if broker_socket: broker_socket.close()
                    broker_socket = None
                continue
            buffer += data
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                if line:
                    try:
                        message = ujson.loads(line.decode('utf-8')); print(f"<- Mensaje del bróker: {message}")
                        if "data" in message and isinstance(message["data"], dict):
                            data_payload = message["data"]; action = data_payload.get("action")
                            if action == "create":
                                seq = data_payload.get("sequence", {}); name = seq.get("name")
                                if name: secuencias_guardadas[name] = seq; print(f"  - Secuencia '{name}' guardada.")
                                else: print("  - ERROR: 'create' no tiene nombre de secuencia.")
                            elif action == "execute_now":
                                name = data_payload.get("name")
                                if name in secuencias_guardadas:
                                    print(f"  - Ejecutando secuencia guardada '{name}'."); ejecutar_secuencia_json(secuencias_guardadas[name])
                                else: print(f"  - ERROR: No se encontró la secuencia '{name}'.")
                            elif action == "schedule_at":
                                name = data_payload.get("name"); time_str = data_payload.get("time")
                                if name and time_str:
                                    try:
                                        h, m, s = [int(p) for p in time_str.split(':')]
                                        now_tuple = time.localtime(); target_tuple = (now_tuple[0], now_tuple[1], now_tuple[2], h, m, s, now_tuple[6], now_tuple[7])
                                        trigger_timestamp = time.mktime(target_tuple)
                                        if trigger_timestamp < time.time(): trigger_timestamp += 86400
                                        tareas_programadas.append({'name': name, 'trigger_timestamp': trigger_timestamp})
                                        fecha_ejecucion = time.localtime(trigger_timestamp)
                                        print(f"  - Tarea '{name}' programada para {fecha_ejecucion[0]}-{fecha_ejecucion[1]:02d}-{fecha_ejecucion[2]:02d} a las {h:02d}:{m:02d}:{s:02d}.")
                                    except (ValueError, IndexError): print("  - ERROR: Formato de hora inválido. Usar 'HH:MM:SS'.")
                                else: print("  - ERROR: 'schedule_at' incompleto (falta 'name' o 'time').")
                            elif action == "schedule_datetime":
                                name = data_payload.get("name"); datetime_str = data_payload.get("datetime")
                                if name and datetime_str:
                                    try:
                                        date_part, time_part = datetime_str.split(' '); Y, M, D = [int(p) for p in date_part.split('-')]
                                        h, m, s = [int(p) for p in time_part.split(':')]; now_tuple = time.localtime()
                                        target_tuple = (Y, M, D, h, m, s, now_tuple[6], now_tuple[7])
                                        trigger_timestamp = time.mktime(target_tuple)
                                        tareas_programadas.append({'name': name, 'trigger_timestamp': trigger_timestamp})
                                        print(f"  - Tarea '{name}' programada para {Y}-{M:02d}-{D:02d} a las {h:02d}:{m:02d}:{s:02d}.")
                                    except (ValueError, IndexError): print("  - ERROR: Formato de fecha/hora inválido. Usar 'YYYY-MM-DD HH:MM:SS'.")
                                else: print("  - ERROR: 'schedule_datetime' incompleto (falta 'name' o 'datetime').")
                            elif action == "set_servos":
                                angles = data_payload.get("angles", {}); logical_s1 = angles.get("s1", angulo_servo1); logical_s2 = angles.get("s2", angulo_servo2); logical_s3 = angles.get("s3", angulo_servo3)
                                physical_s1 = 90 - logical_s1; physical_s2 = logical_s2; physical_s3 = logical_s3 + 95
                                print(f"  - Moviendo servos a (Físicos: {physical_s1}, {physical_s2}, {physical_s3})"); mover_servo_tiempo(physical_s1, physical_s2, physical_s3, 0)
                            elif action is None and "v" in data_payload:
                                ejecutar_estado(data_payload)
                            else: print("   (Acción desconocida o no reconocida)")
                        else: print("   (Mensaje informativo del bróker)")
                    except Exception as e: print(f"Error procesando JSON: {e}")
        except OSError as e:
            if e.args[0] == errno.ETIMEDOUT: pass
            else:
                print(f"Error de red: {e}")
                with broker_lock:
                    if broker_socket: broker_socket.close()
                    broker_socket = None
        except Exception as e:
            print(f"Error inesperado: {e}")
            with broker_lock:
                if broker_socket: broker_socket.close()
                broker_socket = None
        time.sleep(0.5)

# ============================================================
# 8. SERVIDOR PRINCIPAL
# ============================================================
addr = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(addr); s.listen(1)
print("Servidor web listo en", wlan.ifconfig()[0])
_thread.start_new_thread(broker_listener_thread, ())

while True:
    try:
        cl, addr = s.accept(); print("Cliente web conectado:", addr)
        request = cl.recv(1024)
        if b"Upgrade: websocket" in request:
            if websocket_handshake(cl, request):
                while True:
                    msg = recv_ws_frame(cl)
                    if msg is None: break
                    print("Comando WebSocket:", msg)
                    
                    if "forward_start" in msg: mover_robot_simple(80, 80)
                    elif "backward_start" in msg: mover_robot_simple(-80, -80)
                    elif "left_start" in msg: mover_robot_simple(-80, 80)
                    elif "right_start" in msg: mover_robot_simple(80, -80)
                    elif "_stop" in msg: mover_robot_simple(0, 0)
                    elif msg.startswith("servo1_"): mover_servo(servo1, int(msg.split("_")[1]))
                    elif msg.startswith("servo2_"): mover_servo(servo2, int(msg.split("_")[1]))
                    elif msg.startswith("servo3_"): mover_servo(servo3, int(msg.split("_")[1]))
                    elif msg.startswith("subscribe_"): topic = msg.split("_", 1)[1]; subscribe_to_topic(topic)
        else:
            cl.sendall(b"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" + HTML_PAGE.encode())
        cl.close()
    except Exception as e:
        print(f"Error en servidor principal: {e}")
        detener_motores()