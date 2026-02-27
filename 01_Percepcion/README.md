# 🧠 Percepción - Simulación de los Sentidos Humanos

## Propósito

Esta carpeta implementa la **percepción multimodal** del robot humanoide, simulando los sentidos humanos mediante sensores y algoritmos de procesamiento. El objetivo es crear un sistema perceptivo completo que permita al robot entender e interactuar con su entorno de manera similar a un humano.

## 🌐 Visión General: Los 7 Sentidos Robóticos

El robot humanoide integra los siguientes sistemas sensoriales:

| Sentido | Hardware | Software | Carpeta relacionada |
|---------|----------|----------|---------------------|
| 👀 **Visión** | Cámaras RGB + RGB-D | OpenCV, CNNs, YOLO | [`06_Vision/`](../06_Vision/) |
| 👂 **Audición** | Micrófonos array | FFT, MFCC, Whisper, NLP | `audio/` |
| 👃 **Olfato** | Sensores de gas (MQ) | Clasificación ML | `smell/` |
| 👅 **Gusto** | Biosensores químicos | Análisis de composición | `taste/` |
| ✋ **Tacto** | Sensores hápticos/FSR | RNN, LSTM | `touch/` |
| 🧭 **Propiocepción** | IMU, encoders | Filtros Kalman, PID | `proprioception/` |
| ⚖️ **Equilibrio** | IMU, giroscopio | Control dinámico | [`04_Control/`](../04_Control/) |

---

## 📂 Estructura del Directorio

```
01_Percepcion/
├── README.md (este archivo)
├── sensors/                    # Drivers de sensores
│   ├── vision/                # Cámaras y depth sensors
│   ├── audio/                 # Micrófonos y arrays
│   ├── chemical/              # Sensores de gas y químicos
│   ├── tactile/               # Sensores de fuerza y presión
│   ├── imu/                   # IMU, giroscopios, acelerómetros
│   └── temperature/           # Sensores de temperatura
├── fusion/                     # Fusión sensorial multimodal
│   ├── ekf/                   # Extended Kalman Filter
│   ├── ukf/                   # Unscented Kalman Filter
│   ├── complementary/         # Filtros complementarios
│   └── multimodal/            # Fusión de múltiples modalidades
├── preprocessing/              # Preprocesamiento de señales
│   ├── calibration/           # Calibración de sensores
│   ├── filtering/             # Filtrado de ruido
│   ├── normalization/         # Normalización de datos
│   └── feature_extraction/    # Extracción de características
├── notebooks/                  # Notebooks de demostración
│   ├── vision_tests.ipynb
│   ├── audio_processing.ipynb
│   ├── sensor_fusion.ipynb
│   └── multimodal_perception.ipynb
└── models/                     # Modelos entrenados
    ├── vision/
    ├── audio/
    └── fusion/
```

---

## 1. 👀 Visión (Vista)

### Qué se simula
- Detección de objetos y personas
- Reconocimiento de patrones y texturas
- Percepción de profundidad (estereovisión)
- Seguimiento de movimiento
- Reconocimiento facial

### Hardware recomendado
- **Cámaras RGB**: Raspberry Pi Camera V2, Logitech C920
- **Cámaras de profundidad**: Intel RealSense D435i, Kinect v2
- **Estéreo**: Dual Pi Camera setup
- **Resolución**: Al menos 720p @ 30fps

### Software y algoritmos

#### Bibliotecas principales
```python
import cv2                    # OpenCV para visión básica
import numpy as np
from ultralytics import YOLO  # Detección de objetos
import torch
from torchvision import models
import pyrealsense2 as rs     # Para RealSense
```

#### Detección de objetos (YOLO)
```python
# Cargar modelo YOLO
model = YOLO('yolov8n.pt')

# Detectar objetos en tiempo real
results = model(frame, conf=0.5)
for r in results:
    boxes = r.boxes
    for box in boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
```

#### Percepción de profundidad
```python
# Procesamiento de imágenes estéreo
stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
disparity = stereo.compute(img_left_gray, img_right_gray)
depth = baseline * focal_length / (disparity + 1e-10)
```

### Aplicaciones
- Navegación autónoma
- Manipulación de objetos
- Reconocimiento de gestos
- Lectura de texto (OCR)

**Ver más detalles en**: [`06_Vision/`](../06_Vision/)

---

## 2. 👂 Audición (Oído)

### Qué se simula
- Reconocimiento de sonidos y palabras
- Localización espacial de fuentes sonoras
- Interpretación de lenguaje natural
- Detección de emociones en voz
- Separación de fuentes (cocktail party problem)

### Hardware recomendado
- **Micrófonos**: Electret, MEMS
- **Array de micrófonos**: ReSpeaker Mic Array v2.0 (4-6 micrófonos)
- **Frecuencia de muestreo**: 16 kHz mínimo, 44.1 kHz ideal
- **ADC**: 16-bit mínimo

### Software y algoritmos

#### Bibliotecas principales
```python
import pyaudio                  # Captura de audio
import numpy as np
import librosa                  # Análisis de audio
from scipy import signal
import speech_recognition as sr # Speech-to-text
import whisper                  # Modelo de OpenAI
from transformers import pipeline # NLP
```

#### Reconocimiento de voz
```python
# Usando Whisper de OpenAI
import whisper

model = whisper.load_model("base")
result = model.transcribe("audio.wav", language="es")
print(result["text"])
```

#### Procesamiento de señal (FFT y MFCC)
```python
import librosa
import numpy as np

# Cargar audio
y, sr = librosa.load('audio.wav', sr=16000)

# Extraer MFCCs (Mel-Frequency Cepstral Coefficients)
mfccs = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=13)

# FFT para análisis espectral
fft = np.fft.fft(y)
freq = np.fft.fftfreq(len(y), 1/sr)
```

#### Localización de sonido con array de micrófonos
```python
# Beamforming para localización de sonido
def beamforming(signals, fs, direction_angle):
    """
    signals: array de señales de cada micrófono
    fs: frecuencia de muestreo
    direction_angle: ángulo de la fuente sonora
    """
    c = 343  # velocidad del sonido (m/s)
    d = 0.05  # distancia entre micrófonos (m)
    
    delays = []
    for i in range(len(signals)):
        delay = i * d * np.sin(np.radians(direction_angle)) / c
        delays.append(delay)
    
    # Aplicar delays y sumar
    output = np.zeros_like(signals[0])
    for i, signal_data in enumerate(signals):
        shifted = np.roll(signal_data, int(delays[i] * fs))
        output += shifted
    
    return output / len(signals)
```

### Aplicaciones
- Asistente conversacional
- Control por voz
- Detección de palabras clave (wake word)
- Análisis de sentimiento vocal
- Localización de personas que hablan

---

## 3. 👃 Olfato (Nariz Electrónica)

### Qué se simula
- Detección de gases peligrosos (CO, CO2, metano)
- Identificación de químicos volátiles
- Detección de humo e incendios
- Análisis de calidad del aire

### Hardware recomendado
- **Sensores de gas MQ**: MQ-2 (humo), MQ-3 (alcohol), MQ-7 (CO), MQ-135 (calidad aire)
- **E-nose avanzado**: AMS CCS811, BME680
- **ADC**: Para conversión analógica-digital
- **Ubicación**: Cerca de la "nariz" del robot

### Software y algoritmos

#### Biblioteca de lectura
```python
import board
import busio
import adafruit_ccs811
import adafruit_bme680

# Lectura de sensor CCS811 (VOC y eCO2)
i2c = busio.I2C(board.SCL, board.SDA)
ccs811 = adafruit_ccs811.CCS811(i2c)

while True:
    if ccs811.data_ready:
        print(f"CO2: {ccs811.eco2} ppm")
        print(f"TVOC: {ccs811.tvoc} ppb")
```

#### Clasificación con Machine Learning
```python
from sklearn.ensemble import RandomForestClassifier
import numpy as np

# Dataset: [MQ2, MQ3, MQ7, MQ135] -> clase de gas
X_train = np.array([
    [200, 100, 50, 300],   # Aire limpio
    [800, 150, 60, 400],   # Humo
    [300, 600, 55, 350],   # Alcohol
    [250, 120, 500, 380],  # CO
])
y_train = ['clean', 'smoke', 'alcohol', 'co']

clf = RandomForestClassifier()
clf.fit(X_train, y_train)

# Predecir
sensor_readings = [350, 140, 70, 320]
prediction = clf.predict([sensor_readings])
print(f"Gas detectado: {prediction[0]}")
```

### Aplicaciones
- **Seguridad**: Detección de fugas de gas, incendios
- **Salud**: Monitoreo de calidad del aire interior
- **Industrial**: Control de procesos químicos
- **Búsqueda y rescate**: Localización de personas en incendios

---

## 4. 👅 Gusto (Sensores Químicos)

### Qué se simula
- Identificación de sustancias y materiales
- Análisis de pH y conductividad
- Detección de contaminantes en líquidos
- Análisis de composición química

### Hardware recomendado
- **Sensores de pH**: Electrodo de pH + módulo de lectura
- **TDS sensor**: Total Dissolved Solids
- **Conductividad**: EC meter
- **Biosensores**: Para aplicaciones específicas

### Software y algoritmos

```python
# Lectura de sensor de pH
class PHSensor:
    def __init__(self, pin):
        self.pin = pin
        self.calibration_offset = 0.0
    
    def read_ph(self):
        voltage = self.read_voltage()
        ph = 3.5 * voltage + self.calibration_offset
        return ph
    
    def classify_substance(self, ph):
        if ph < 6.5:
            return "Ácido"
        elif ph > 7.5:
            return "Básico"
        else:
            return "Neutro"
```

### Aplicaciones
- **Robots industriales**: Control de calidad en alimentos/bebidas
- **Biomédico**: Análisis de muestras
- **Ambiental**: Monitoreo de calidad del agua
- **Menos común en robots de servicio domésticos**

---

## 5. ✋ Tacto (Piel Electrónica)

### Qué se simula
- Detección de presión y contacto
- Sensación de textura
- Medición de temperatura superficial
- Detección de dolor (fuerza excesiva)

### Hardware recomendado
- **FSR (Force Sensitive Resistors)**: Detección de presión
- **Sensores capacitivos**: Detección de proximidad y toque ligero
- **Piezoeléctricos**: Vibración y golpes
- **MLX90614**: Temperatura sin contacto (IR)
- **Skin patches**: Arrays de sensores táctiles

### Software y algoritmos

#### Lectura de sensores de fuerza
```python
import RPi.GPIO as GPIO
import spidev

class TactileSensor:
    def __init__(self, num_sensors=16):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.num_sensors = num_sensors
        
    def read_pressure_matrix(self):
        """Lee matriz de sensores de presión"""
        pressure_map = []
        for i in range(self.num_sensors):
            value = self.spi.xfer2([1, (8 + i) << 4, 0])
            pressure = ((value[1] & 3) << 8) + value[2]
            pressure_map.append(pressure)
        return np.array(pressure_map).reshape(4, 4)
    
    def detect_touch_location(self, threshold=100):
        pressure = self.read_pressure_matrix()
        touch_points = np.argwhere(pressure > threshold)
        return touch_points
```

#### Clasificación de texturas con RNN/LSTM
```python
import torch
import torch.nn as nn

class TextureClassifier(nn.Module):
    def __init__(self, input_size=16, hidden_size=64, num_classes=5):
        super().__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, batch_first=True)
        self.fc = nn.Linear(hidden_size, num_classes)
    
    def forward(self, x):
        # x: (batch, sequence_length, input_size)
        lstm_out, _ = self.lstm(x)
        out = self.fc(lstm_out[:, -1, :])  # Última salida
        return out

# Texturas: suave, rugoso, duro, blando, frío
textures = ['smooth', 'rough', 'hard', 'soft', 'cold']
```

### Aplicaciones
- **Manipulación delicada**: Agarre de objetos frágiles
- **Interacción social**: Detección de apretones de manos, contacto humano
- **Seguridad**: Detección de colisiones
- **Exploración**: Identificación de superficies

---

## 6. 🧭 Propiocepción (Conciencia Corporal)

### Qué se simula
- Posición de todas las articulaciones
- Orientación del cuerpo en el espacio
- Velocidad angular y aceleración
- Centro de masa y equilibrio

### Hardware recomendado
- **IMU**: MPU6050, BNO055, BMI160
- **Encoders**: En cada articulación (absolutos o incrementales)
- **Giroscopios**: Integrados en IMU
- **Acelerómetros**: 3 ejes
- **Magnetómetro**: Para orientación absoluta (opcional)

### Software y algoritmos

#### Lectura de IMU
```python
import smbus
import math

class IMU:
    def __init__(self, address=0x68):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.init_mpu6050()
    
    def read_accelerometer(self):
        # Leer registros de acelerómetro
        acc_x = self.read_word_2c(0x3B) / 16384.0
        acc_y = self.read_word_2c(0x3D) / 16384.0
        acc_z = self.read_word_2c(0x3F) / 16384.0
        return acc_x, acc_y, acc_z
    
    def read_gyroscope(self):
        # Leer registros de giroscopio
        gyro_x = self.read_word_2c(0x43) / 131.0
        gyro_y = self.read_word_2c(0x45) / 131.0
        gyro_z = self.read_word_2c(0x47) / 131.0
        return gyro_x, gyro_y, gyro_z
    
    def get_orientation(self):
        acc_x, acc_y, acc_z = self.read_accelerometer()
        roll = math.atan2(acc_y, acc_z) * 57.2958  # rad a grados
        pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 57.2958
        return roll, pitch
```

#### Filtro Complementario (fusión acelerómetro + giroscopio)
```python
class ComplementaryFilter:
    def __init__(self, alpha=0.98, dt=0.01):
        self.alpha = alpha  # Peso del giroscopio
        self.dt = dt
        self.roll = 0.0
        self.pitch = 0.0
    
    def update(self, acc_x, acc_y, acc_z, gyro_x, gyro_y):
        # Calcular ángulos del acelerómetro
        acc_roll = math.atan2(acc_y, acc_z) * 57.2958
        acc_pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 57.2958
        
        # Integrar giroscopio
        self.roll = self.alpha * (self.roll + gyro_x * self.dt) + (1 - self.alpha) * acc_roll
        self.pitch = self.alpha * (self.pitch + gyro_y * self.dt) + (1 - self.alpha) * acc_pitch
        
        return self.roll, self.pitch
```

#### Control PID para articulaciones
```python
class JointController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def compute(self, setpoint, measured_position, dt):
        error = setpoint - measured_position
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        
        return output
```

### Aplicaciones
- **Locomoción bípeda**: Control de equilibrio al caminar
- **Manipulación**: Control preciso de brazos
- **Recuperación de caídas**: Detección y reacción
- **Navegación**: Dead reckoning (estimación de posición)

**Ver más en**: [`04_Control/`](../04_Control/)

---

## 7. ⚖️ Equilibrio (Vestibular System)

### Qué se simula
- Detección de inclinación y caída
- Estabilización dinámica
- Respuesta a perturbaciones externas
- Control de centro de gravedad

### Hardware
- Mismo que propiocepción (IMU principalmente)
- **Sensores de presión en pies**: Para detección de ZMP (Zero Moment Point)

### Algoritmos de control

#### Detección de caída inminente
```python
def detect_fall(roll, pitch, threshold=30):
    """Detecta si el robot va a caer"""
    if abs(roll) > threshold or abs(pitch) > threshold:
        return True, "¡Caída inminente!"
    return False, "Estable"
```

#### Control de ZMP
```python
class ZMPController:
    """Control del Zero Moment Point para estabilidad"""
    def __init__(self):
        self.com_height = 0.5  # Centro de masa (m)
        self.g = 9.81
    
    def calculate_zmp(self, com_x, com_y, acc_x, acc_y):
        """
        Calcula la posición del ZMP
        Si ZMP está fuera del polígono de soporte, el robot cae
        """
        zmp_x = com_x - (self.com_height / self.g) * acc_x
        zmp_y = com_y - (self.com_height / self.g) * acc_y
        return zmp_x, zmp_y
    
    def is_stable(self, zmp_x, zmp_y, support_polygon):
        """Verifica si ZMP está dentro del polígono de soporte"""
        # support_polygon: lista de puntos [(x1,y1), (x2,y2), ...]
        # Usar algoritmo de punto en polígono
        return point_in_polygon((zmp_x, zmp_y), support_polygon)
```

---

## 🔗 Fusión Sensorial Multimodal

La verdadera potencia de la percepción robótica surge al combinar múltiples modalidades sensoriales:

### Extended Kalman Filter (EKF)
```python
import numpy as np

class EKF:
    def __init__(self, state_dim, measurement_dim):
        self.x = np.zeros(state_dim)  # Estado
        self.P = np.eye(state_dim)     # Covarianza
        self.Q = np.eye(state_dim) * 0.1  # Ruido del proceso
        self.R = np.eye(measurement_dim) * 0.5  # Ruido de medición
    
    def predict(self, F, B, u):
        """Predicción del estado"""
        self.x = F @ self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, z, H):
        """Actualización con medición"""
        y = z - H @ self.x  # Innovación
        S = H @ self.P @ H.T + self.R  # Covarianza de innovación
        K = self.P @ H.T @ np.linalg.inv(S)  # Ganancia de Kalman
        
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P
```

### Fusión visión + audio (detección de persona hablando)
```python
class MultimodalFusion:
    def __init__(self):
        self.vision_weight = 0.6
        self.audio_weight = 0.4
    
    def fuse_person_detection(self, vision_bbox, audio_direction):
        """
        Combina detección visual con localización de audio
        para confirmar que una persona está hablando
        """
        if vision_bbox and audio_direction:
            # Calcular ángulo de la bounding box
            bbox_center_x = (vision_bbox[0] + vision_bbox[2]) / 2
            image_center = 320  # Asumiendo imagen de 640px
            bbox_angle = (bbox_center_x - image_center) / image_center * 45  # FOV de 90°
            
            # Comparar con dirección del audio
            angle_diff = abs(bbox_angle - audio_direction)
            
            if angle_diff < 15:  # 15 grados de tolerancia
                confidence = 0.95
                return True, confidence, "Persona hablando detectada"
            else:
                confidence = 0.5
                return False, confidence, "Discrepancia visión-audio"
        
        return False, 0.0, "Datos insuficientes"
```

---

## 📊 Integración con ROS

Todos los sensores se integran mediante topics de ROS:

```bash
# Topics de sensores
/perception/vision/objects          # Objetos detectados
/perception/audio/transcript        # Transcripción de voz
/perception/audio/sound_location    # Localización de sonido
/perception/smell/gas_levels        # Niveles de gases
/perception/touch/contact_points    # Puntos de contacto
/perception/imu/data                # Datos de IMU
/perception/balance/zmp             # Zero Moment Point
/perception/fusion/multimodal       # Fusión de todas las modalidades
```

---

## 📚 Referencias y Recursos

### Librerías principales
- **OpenCV**: `pip install opencv-python opencv-contrib-python`
- **PyTorch**: `pip install torch torchvision`
- **Librosa**: `pip install librosa`
- **Whisper**: `pip install openai-whisper`
- **ROS drivers**: `rosdep install sensor_msgs geometry_msgs`

### Papers recomendados
1. "A Survey on Multisensory Fusion for Mobile Robots" (IEEE)
2. "Tactile Sensing for Robot Manipulation" (Robotics Science and Systems)
3. "Audio-Visual Scene Understanding" (CVPR)
4. "Electronic Nose Systems for Robotic Applications" (Sensors Journal)

### Datasets
- **Visión**: COCO, ImageNet, KITTI
- **Audio**: LibriSpeech, Common Voice, AudioSet
- **Táctil**: STAG (Surface Texture and Geometry)
- **Multimodal**: EPIC-KITCHENS, Ego4D

---

## 🎯 Próximos Pasos

1. **Implementar cada modalidad sensorial individualmente**
2. **Calibrar todos los sensores**
3. **Desarrollar pipeline de fusión sensorial**
4. **Crear sistema de visualización en RViz**
5. **Entrenar modelos de ML para clasificación**
6. **Integrar con sistema de control** ([`04_Control/`](../04_Control/))

---

**Ver también**:
- [📚 Recursos de Conocimiento](../00_Gestion_Proyecto/recursos_conocimientos.md)
- [🤖 Visión por Computadora](../06_Vision/)
- [🎮 Control](../04_Control/)
- [🧠 Aprendizaje de Máquina](../05_Aprendizaje_Maquina/)