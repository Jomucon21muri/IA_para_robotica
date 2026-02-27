# 🛡️ Ética, Seguridad y Buenas Prácticas - Robot Humanoide

## Propósito

Garantizar que el desarrollo y operación del robot humanoide sean seguros, éticos y responsables. Este módulo es CRÍTICO y NO OPCIONAL.

## Área de Conocimiento

Este módulo implementa el pilar 11: **Seguridad y Ética** (ver [recursos_conocimientos.md](../00_Gestion_Proyecto/recursos_conocimientos.md)).

## Estructura del Directorio

```
12_Etica_Seguridad/
├── README.md (este archivo)
├── risk_assessment/
│   ├── fmea_analisis.md        # Failure Mode and Effects Analysis
│   ├── riesgos_mecanicos.md
│   ├── riesgos_electricos.md
│   └── riesgos_software.md
├── safety_checklists/
│   ├── pre_prueba.md           # Checklist obligatorio pre-prueba
│   ├── operacion_diaria.md
│   ├── mantenimiento.md
│   └── transporte.md
├── safety_procedures/
│   ├── emergency_stop.md       # Procedimiento de parada de emergencia
│   ├── battery_handling.md     # Manejo seguro de baterías LiPo
│   ├── electrical_safety.md
│   └── physical_safety.md
├── ethics/
│   ├── principios_eticos.md    # Principios éticos del proyecto
│   ├── privacidad_datos.md     # Política de privacidad
│   ├── ai_ethics.md            # Ética en IA
│   └── compliance.md           # Normativas y estándares
├── incident_reports/
│   ├── template.md             # Plantilla de reporte
│   └── 2026-02-27_example.md   # Ejemplo
├── standards/
│   ├── iso_15066.md            # Robots colaborativos
│   ├── iso_13482.md            # Robots de cuidado personal
│   └── safety_limits.md        # Límites de seguridad definidos
└── documentation/
    ├── safety_manual.md        # Manual de seguridad completo
    └── training_materials.md   # Material de capacitación
```

## ⚠️ Principios de Seguridad Fundamentales

### Reglas de Oro

1. **NUNCA comprometer la seguridad por velocidad de desarrollo**
2. **SIEMPRE tener E-stop físico accesible**
3. **NUNCA dejar robot operando sin supervisión**
4. **SIEMPRE probar en entorno controlado primero**
5. **VERIFICAR lista de seguridad antes de CADA prueba**

### Leyes de la Robótica (Asimov - Filosóficas)

1. Un robot no puede dañar a un ser humano o, por inacción, permitir que un ser humano sufra daño
2. Un robot debe obedecer las órdenes dadas por los seres humanos, excepto si estas órdenes entrasen en conflicto con la Primera Ley
3. Un robot debe proteger su propia existencia en la medida en que esta protección no entre en conflicto con la Primera o la Segunda Ley

## 🚨 Checklist de Seguridad Pre-Prueba

**⚠️ OBLIGATORIO antes de CADA sesión de pruebas**

### Hardware
- [ ] **Batería**
  - [ ] Voltaje verificado (no sobredescargada)
  - [ ] Conexiones aseguradas
  - [ ] BMS funcionando
  - [ ] Sin hinchazón visible
  
- [ ] **E-stop (Parada de Emergencia)**
  - [ ] Botón físico accesible
  - [ ] Probado y funcional
  - [ ] Corta alimentación a motores
  
- [ ] **Estructura Mecánica**
  - [ ] Sin grietas o daños visibles
  - [ ] Tornillos apretados
  - [ ] Articulaciones se mueven libremente
  - [ ] No hay piezas sueltas
  
- [ ] **Cableado**
  - [ ] Sin cables pelados o expuestos
  - [ ] Conexiones seguras
  - [ ] Cable management ordenado
  - [ ] Sin signos de sobrecalentamiento

### Software
- [ ] **Límites de Movimiento**
  - [ ] Rangos articulares configurados
  - [ ] Software limits implementados
  - [ ] Validados en prueba lenta
  
- [ ] **Emergency Stop en Software**
  - [ ] Implementado en código
  - [ ] Probado
  - [ ] Timeout de watchdog configurado
  
- [ ] **Logging**
  - [ ] Sistema de logs activado
  - [ ] Grabando datos importantes
  
- [ ] **Comunicación**
  - [ ] Conexión estable verificada
  - [ ] No hay lag significativo
  - [ ] Fallback si se pierde conexión

### Entorno
- [ ] **Área de Prueba**
  - [ ] Espacio despejado (mín. 3m x 3m)
  - [ ] Superficie plana
  - [ ] Sin personas u objetos frágiles cerca
  - [ ] Buena iluminación
  
- [ ] **Personal**
  - [ ] Solo personal autorizado presente
  - [ ] Todos conocen ubicación de E-stop
  - [ ] Comunicación clara establecida
  
- [ ] **Equipamiento de Seguridad**
  - [ ] Extintor accesible (para baterías LiPo)
  - [ ] Kit de primeros auxilios disponible
  - [ ] Gafas de protección (si aplica)

### Procedimiento
- [ ] **Plan de Prueba Definido**
  - [ ] Objetivos claros
  - [ ] Secuencia de pasos
  - [ ] Criterios de éxito/fallo
  
- [ ] **Progresión Gradual**
  - [ ] Comenzar con movimientos lentos
  - [ ] Un sistema a la vez
  - [ ] Incrementar complejidad gradualmente

## 📋 Procedimientos de Emergencia

### Parada de Emergencia

**Si algo va mal**:
1. **PRESIONAR E-STOP inmediatamente**
2. No intentar "salvar" el robot - seguridad primero
3. Desconectar batería si hay riesgo eléctrico
4. Evaluar situación antes de reiniciar
5. Registrar incidente

### Incendio de Batería LiPo

**⚠️ LiPo en llamas no pueden extinguirse con agua**

1. **Activar E-stop**
2. **Evacuar área inmediata**
3. **Usar extintor tipo D** (para metales) o arena
4. Si no es seguro: Dejar que se consuma en área segura
5. **NO inhalar humo** (extremadamente tóxico)
6. Llamar a emergencias si es severo

**Prevención**:
- Nunca sobrecargar o sobredescargar
- Almacenar en bolsa LiPo resistente al fuego
- Cargar con supervisión
- Desechar si está hinchada o dañada

### Choque Eléctrico

1. **NO tocar a la persona** si aún está en contacto
2. **Desconectar alimentación** (E-stop, desenchufar)
3. Llamar a emergencias si es severo
4. Primeros auxilios si está capacitado

**Prevención**:
- No trabajar en circuitos energizados innecesariamente
- Aislar componentes de alto voltaje
- Usar herramientas aisladas

### Daño Físico (Robot golpea algo/alguien)

1. **E-stop inmediato**
2. Evaluar daños (personas > robot)
3. Primeros auxilios si necesario
4. Análisis de causa raíz
5. Implementar mitigaciones antes de continuar

## 🔒 Seguridad en Software

### Límites de Seguridad

**Implementar en TODOS los controladores**:
```python
class SafeJoint:
    """
    Articulación con límites de seguridad
    """
    def __init__(self, name, min_angle, max_angle, max_speed, max_torque):
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.max_speed = max_speed  # deg/s
        self.max_torque = max_torque  # N·m
        self.current_angle = 0
        self.emergency_stop = False
    
    def move_to(self, target_angle, speed):
        """
        Mueve con validación de seguridad
        """
        if self.emergency_stop:
            logger.error(f"{self.name}: E-stop activado")
            return False
        
        # Validar rango
        if not (self.min_angle <= target_angle <= self.max_angle):
            logger.error(f"{self.name}: Ángulo {target_angle} fuera de rango [{self.min_angle}, {self.max_angle}]")
            return False
        
        # Validar velocidad
        if abs(speed) > self.max_speed:
            logger.warning(f"{self.name}: Velocidad {speed} excede máximo {self.max_speed}, limitando")
            speed = np.sign(speed) * self.max_speed
        
        # Aplicar movimiento
        self.current_angle = target_angle
        return True
    
    def estop(self):
        """Parada de emergencia"""
        self.emergency_stop = True
        logger.critical(f"{self.name}: EMERGENCY STOP")
        # Detener motor inmediatamente
        self.motor.stop()
```

### Watchdog Timer

**Detectar software colgado**:
```python
import threading
import time

class Watchdog:
    """
    Watchdog para detectar si el sistema se cuelga
    """
    def __init__(self, timeout=2.0, callback=None):
        self.timeout = timeout
        self.callback = callback or self.default_callback
        self.last_kick = time.time()
        self.running = True
        
        self.thread = threading.Thread(target=self._monitor, daemon=True)
        self.thread.start()
    
    def kick(self):
        """Reset watchdog - llamar regularmente en loop principal"""
        self.last_kick = time.time()
    
    def _monitor(self):
        while self.running:
            if time.time() - self.last_kick > self.timeout:
                logger.critical("WATCHDOG TIMEOUT - Sistema no responde")
                self.callback()
            time.sleep(0.1)
    
    def default_callback(self):
        """Acción por defecto si timeout"""
        # Activar E-stop
        robot.emergency_stop()
        # Enviar alerta
        send_alert("Watchdog timeout - robot detenido")
    
    def stop(self):
        self.running = False

# Uso en loop principal
watchdog = Watchdog(timeout=2.0)

while True:
    watchdog.kick()  # Indicar que estamos vivos
    
    # Procesamiento normal
    robot.update()
    
    time.sleep(0.01)  # 100 Hz
```

### Validación de Inputs

**Nunca confiar en inputs externos**:
```python
def validate_command(command):
    """
    Valida comando antes de ejecutar
    """
    # Verificar estructura
    if not isinstance(command, dict):
        return False, "Comando debe ser diccionario"
    
    if 'type' not in command:
        return False, "Falta campo 'type'"
    
    # Validar tipo de comando
    valid_types = ['move', 'stop', 'calibrate', 'status']
    if command['type'] not in valid_types:
        return False, f"Tipo '{command['type']}' no válido"
    
    # Validar parámetros específicos
    if command['type'] == 'move':
        if 'joint' not in command or 'angle' not in command:
            return False, "Comando move requiere 'joint' y 'angle'"
        
        # Validar rango (pre-check antes de enviar a SafeJoint)
        joint_limits = get_joint_limits(command['joint'])
        if not (joint_limits['min'] <= command['angle'] <= joint_limits['max']):
            return False, f"Ángulo fuera de rango para {command['joint']}"
    
    return True, "OK"

# Uso
received_command = {'type': 'move', 'joint': 'elbow', 'angle': 45}

valid, message = validate_command(received_command)
if valid:
    execute_command(received_command)
else:
    logger.warning(f"Comando rechazado: {message}")
```

## 🤝 Ética en IA y Robótica

### Principios Éticos del Proyecto

1. **Transparencia**: El comportamiento del robot debe ser predecible y explicable
2. **Equidad**: No discriminar por género, edad, etnia, etc.
3. **Privacidad**: Respetar datos personales
4. **Responsabilidad**: Hacernos responsables de las acciones del robot
5. **Beneficencia**: Diseñar para ayudar, no dañar

### Privacidad de Datos

**Datos sensibles a proteger**:
- Imágenes/video de caras → **Anonimizar o pedir consentimiento**
- Grabaciones de audio → **Solo con consentimiento**
- Ubicación del hogar → **No compartir**
- Patrones de comportamiento → **Anonimizar**

**Implementación**:
```python
import hashlib
import cv2

def anonymize_face(image):
    """
    Difumina rostros en imagen
    """
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(image, 1.3, 5)
    
    for (x, y, w, h) in faces:
        # Difuminar rostro
        face_region = image[y:y+h, x:x+w]
        blurred = cv2.GaussianBlur(face_region, (99, 99), 30)
        image[y:y+h, x:x+w] = blurred
    
    return image

def collect_telemetry(user_id, robot_state):
    """
    Recolecta telemetría sin datos personales
    """
    # Anonimizar ID de usuario
    anon_id = hashlib.sha256(user_id.encode()).hexdigest()
    
    telemetry = {
        'user': anon_id,
        'timestamp': time.time(),
        'battery_level': robot_state['battery'],
        'position': 'anonymized',  # No guardar posición exacta
        'task_success': robot_state['task_completed']
    }
    
    # Guardar sin información personal identificable
    save_telemetry(telemetry)
```

### Detección de Sesgo en Modelos

**Evaluar equidad**:
```python
def evaluate_model_fairness(model, test_data, sensitive_attributes=['age', 'gender']):
    """
    Evalúa si modelo tiene sesgo respecto a atributos sensibles
    
    Nota: Solo usar atributos sensibles donde sea legal y ético
    """
    results = {}
    
    for attr in sensitive_attributes:
        groups = test_data.groupby(attr)
        accuracies = {}
        
        for group_name, group_data in groups:
            predictions = model.predict(group_data['features'])
            accuracy = (predictions == group_data['labels']).mean()
            accuracies[group_name] = accuracy
        
        # Calcular disparidad
        max_acc = max(accuracies.values())
        min_acc = min(accuracies.values())
        disparity = max_acc - min_acc
        
        results[attr] = {
            'accuracies': accuracies,
            'disparity': disparity,
            'fair': disparity < 0.1  # Umbral del 10%
        }
        
        if disparity > 0.1:
            logger.warning(f"Posible sesgo en {attr}: {accuracies}")
    
    return results
```

## 📊 Análisis de Riesgos (FMEA)

### Ejemplo: Failure Mode and Effects Analysis

| ID | Modo de Fallo | Causa | Efecto | Severidad (1-10) | Probabilidad (1-10) | Detección (1-10) | RPN | Mitigación |
|----|---------------|-------|--------|------------------|---------------------|------------------|-----|------------|
| R1 | Servo falla | Sobrecarga, defecto | Pérdida de control articulación | 7 | 4 | 3 | 84 | Monitoreo corriente, redundancia |
| R2 | Software crash | Bug, overflow | Movimiento errático | 9 | 3 | 5 | 135 | Watchdog, testing exhaustivo |
| R3 | Batería se agota | Uso prolongado | Parada súbita | 6 | 7 | 2 | 84 | Monitor batería, alerta temprana |
| R4 | Pérdida WiFi | Interferencia | Pérdida de control | 8 | 5 | 3 | 120 | Failsafe automático, modo autónomo |
| R5 | Sensor IMU falla | Vibración, defecto | Pérdida equilibrio | 9 | 2 | 6 | 108 | Fusión multi-sensor, validación |

**RPN = Severidad × Probabilidad × Detección**
Priorizar si RPN > 100

## 📝 Reporte de Incidentes

### Template

```markdown
# Reporte de Incidente - [FECHA]

## Información General
- **Fecha**: YYYY-MM-DD HH:MM
- **Ubicación**: [Lugar del incidente]
- **Reportado por**: [Nombre]
- **Severidad**: [Baja / Media / Alta / Crítica]

## Descripción del Incidente
[Descripción detallada de qué sucedió]

## Causa Raíz
[Qué causó el incidente - si se conoce]

## Impacto
- **Daños físicos**: [Descripción]
- **Daños al robot**: [Descripción]
- **Tiempo de inactividad**: [X horas]

## Acciones Inmediatas
1. [Acción tomada inmediatamente]
2. [Otra acción]

## Acciones Correctivas
1. [Acción para prevenir recurrencia]
2. [Otra acción]

## Lecciones Aprendidas
- [Lección 1]
- [Lección 2]

## Seguimiento
- [ ] Implementar acción correctiva 1
- [ ] Implementar acción correctiva 2
- [ ] Revisar procedimientos
- [ ] Actualizar documentación
```

## 📚 Normativas y Estándares

### ISO/TS 15066: Robots Colaborativos
- Límites de fuerza y presión en contacto humano-robot
- Movimiento a velocidad segura (<250 mm/s en zona colaborativa)
- Detección de colisión

### ISO 13482: Robots de Cuidado Personal
- Requisitos de seguridad para robots que asisten personas
- Mitigación de riesgos

### ISO 10218: Robots Industriales
- Seguridad general de robots
- Paradas de emergencia
- Modos de operación

### IEC 60950/62368: Seguridad Eléctrica
- Protección contra choques eléctricos
- Aislamiento
- Materiales retardantes de llama

## 🎓 Capacitación Obligatoria

**Antes de operar el robot, cada persona debe**:
1. Leer este documento completo
2. Conocer ubicación y uso de E-stop
3. Conocer procedimientos de emergencia
4. Entender límites del robot
5. Firmar documento de conocimiento (si es proyecto institucional)

## Próximos Pasos

1. **Completar análisis FMEA** para todos los subsistemas
2. **Implementar E-stop** físico y en software
3. **Crear checklist físico** impreso y en ubicación visible
4. **Configurar sistema de logging** de eventos de seguridad
5. **Realizar simulacro** de emergencia

## Referencias

- 📖 "Robot Ethics" - Lin, Abney, Bekey
- 📖 "AI Ethics" - Coeckelbergh
- 🌐 ISO Standards: www.iso.org
- 🌐 EU AI Act
- 🌐 IEEE Ethically Aligned Design
- 🌐 Battery University (seguridad LiPo)

---

**⚠️ RECORDATORIO**: La seguridad NO es opcional. Si hay dudas sobre la seguridad de una operación, DETENER y consultar.

**Última actualización**: Febrero 2026