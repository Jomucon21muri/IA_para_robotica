# 🧠 Aprendizaje Automático y IA - Robot Humanoide

## Propósito

Implementar capacidades de inteligencia artificial en el robot humanoide, incluyendo reconocimiento de objetos, procesamiento de lenguaje natural, aprendizaje por refuerzo para control, y otras técnicas de ML/DL que permitan al robot percibir, aprender y mejorar sus capacidades.

## Área de Conocimiento

Este módulo implementa el pilar 6: **Inteligencia Artificial y Aprendizaje Automático** (ver [recursos_conocimientos.md](../00_Gestion_Proyecto/recursos_conocimientos.md)).

## Estructura del Directorio

```
05_Aprendizaje_Maquina/
├── README.md (este archivo)
├── supervised/
│   ├── clasificacion/       # Clasificación de objetos, gestos
│   ├── deteccion/           # Detección de objetos (YOLO, SSD)
│   ├── segmentacion/        # Segmentación semántica
│   └── regresion/           # Predicción continua (pose estimation)
├── rl/
│   ├── entornos/            # Entornos de simulación (Gym, PyBullet)
│   ├── agentes/             # Implementaciones de agentes (PPO, SAC)
│   ├── politicas/           # Políticas entrenadas
│   └── experimentos/        # Logs, resultados de entrenamiento
├── nlp/
│   ├── speech_recognition/  # Reconocimiento de voz
│   ├── text_to_speech/      # Síntesis de voz
│   ├── nlu/                 # Natural Language Understanding
│   └── conversation/        # Manejo de diálogos
├── imitation/
│   ├── demonstrations/      # Datos de demostración humana
│   ├── behavioral_cloning/  # Clonación de comportamiento
│   └── inverse_rl/          # Reinforcement Learning inverso
├── vision/
│   ├── face_recognition/    # Reconocimiento facial
│   ├── pose_estimation/     # Estimación de pose humana
│   ├── gesture_recognition/ # Reconocimiento de gestos
│   └── tracking/            # Seguimiento de objetos
├── models/
│   ├── pretrained/          # Modelos pre-entrenados
│   ├── finetuned/           # Modelos fine-tuneados
│   └── custom/              # Modelos personalizados
├── datasets/
│   ├── coleccion/           # Scripts de recolección de datos
│   ├── procesamiento/       # Limpieza y augmentation
│   └── splits/              # Train/val/test splits
└── deployment/
    ├── optimization/        # Cuantización, pruning
    ├── inference/           # Código de inferencia
    └── edge/                # Optimización para edge (Jetson)
```

## Aplicaciones de IA en el Robot Humanoide

### 1. Percepción Visual

**Reconocimiento de Objetos**:
- Identificar objetos del entorno
- Modelos: YOLO, EfficientDet
- Uso: Navegación, manipulación

**Reconocimiento Facial**:
- Identificar personas conocidas
- Modelos: FaceNet, ArcFace
- Uso: Interacción personalizada

**Estimación de Pose**:
- Detectar postura de humanos
- Modelos: OpenPose, MediaPipe
- Uso: Imitación de movimientos, interacción

**Ejemplo: Detección de objetos con YOLO**:
```python
import cv2
import torch

# Cargar modelo YOLOv5
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

def detect_objects(image):
    """
    Detecta objetos en imagen usando YOLOv5
    
    Args:
        image: Imagen BGR de OpenCV
    
    Returns:
        results: Detecciones con bbox, clase, confianza
    """
    # Inferencia
    results = model(image)
    
    # Obtener detecciones
    detections = results.pandas().xyxy[0]
    
    return detections

# Uso con cámara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    detections = detect_objects(frame)
    
    # Dibujar resultados
    for _, det in detections.iterrows():
        x1, y1, x2, y2 = int(det['xmin']), int(det['ymin']), int(det['xmax']), int(det['ymax'])
        label = f"{det['name']} {det['confidence']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow('Object Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### 2. Procesamiento de Lenguaje Natural

**Reconocimiento de Voz**:
```python
import speech_recognition as sr

class VoiceRecognizer:
    def __init__(self, language='es-ES'):
        self.recognizer = sr.Recognizer()
        self.language = language
    
    def listen(self, timeout=5):
        """
        Escucha y transcribe comando de voz
        
        Returns:
            str: Texto transcrito o None si falla
        """
        with sr.Microphone() as source:
            print("Escuchando...")
            
            # Ajustar ruido ambiente
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            
            try:
                audio = self.recognizer.listen(source, timeout=timeout)
                text = self.recognizer.recognize_google(audio, language=self.language)
                print(f"Reconocido: {text}")
                return text
            except sr.WaitTimeoutError:
                print("Timeout")
                return None
            except sr.UnknownValueError:
                print("No se pudo entender")
                return None
            except sr.RequestError as e:
                print(f"Error del servicio: {e}")
                return None

# Uso
vr = VoiceRecognizer()
command = vr.listen()

if command:
    # Procesar comando
    if "adelante" in command.lower():
        robot.move_forward()
    elif "atrás" in command.lower():
        robot.move_backward()
```

**Síntesis de Voz (TTS)**:
```python
import pyttsx3

class VoiceSynthesizer:
    def __init__(self, rate=150, volume=0.9):
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', rate)
        self.engine.setProperty('volume', volume)
        
        # Configurar voz en español si está disponible
        voices = self.engine.getProperty('voices')
        for voice in voices:
            if 'spanish' in voice.name.lower():
                self.engine.setProperty('voice', voice.id)
                break
    
    def speak(self, text):
        """
        Sintetiza y reproduce texto
        
        Args:
            text (str): Texto a hablar
        """
        print(f"Diciendo: {text}")
        self.engine.say(text)
        self.engine.runAndWait()
    
    def speak_async(self, text):
        """Habla sin bloquear"""
        self.engine.say(text)
        self.engine.startLoop(False)
        self.engine.iterate()
        self.engine.endLoop()

# Uso
tts = VoiceSynthesizer()
tts.speak("Hola, soy un robot humanoide")
tts.speak("¿En qué puedo ayudarte?")
```

### 3. Aprendizaje por Refuerzo

**Control de Locomoción**:
Entrenar al robot a caminar de forma estable usando RL.

```python
import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

# Crear entorno personalizado
class HumanoidWalkEnv(gym.Env):
    """
    Entorno de marcha para robot humanoide
    """
    def __init__(self):
        super(HumanoidWalkEnv, self).__init__()
        
        # Espacio de observación: posiciones articulares, velocidades, IMU
        self.observation_space = gym.spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(40,),  # Ejemplo
            dtype=np.float32
        )
        
        # Espacio de acción: torques/ángulos de articulaciones
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),  # 12 articulaciones principales
            dtype=np.float32
        )
        
        self.robot = None  # Conexión con simulador o robot real
    
    def reset(self):
        # Resetear robot a posición inicial
        state = self.robot.reset()
        return state
    
    def step(self, action):
        # Aplicar acción
        self.robot.apply_action(action)
        
        # Obtener nuevo estado
        state = self.robot.get_state()
        
        # Calcular recompensa
        reward = self._calculate_reward(state, action)
        
        # Verificar si terminó
        done = self._check_done(state)
        
        info = {}
        
        return state, reward, done, info
    
    def _calculate_reward(self, state, action):
        """
        Función de recompensa:
        - Avanzar hacia adelante: +
        - Mantener equilibrio: +
        - Consumo energético: -
        - Caída: - -
        """
        forward_velocity = state['velocity_x']
        height = state['torso_height']
        energy = np.sum(np.abs(action))
        
        reward = 0.0
        reward += forward_velocity * 10  # Incentivar avance
        reward += max(0, height - 0.8) * 5  # Mantener altura
        reward -= energy * 0.01  # Penalizar gasto energético
        
        if height < 0.5:  # Caída
            reward -= 100
        
        return reward
    
    def _check_done(self, state):
        """Terminar si se cae o alcanza objetivo"""
        if state['torso_height'] < 0.5:
            return True
        if state['position_x'] > 10.0:  # Meta
            return True
        return False

# Entrenamiento
env = make_vec_env(HumanoidWalkEnv, n_envs=4)

# Crear agente PPO
model = PPO(
    'MlpPolicy',
    env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    tensorboard_log="./logs/"
)

# Entrenar
model.learn(total_timesteps=1_000_000)

# Guardar modelo
model.save("humanoid_walk_ppo")

# Evaluar
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    if done:
        obs = env.reset()
```

### 4. Aprendizaje por Imitación

**Clonación de Comportamiento**:
Aprender acciones a partir de demostraciones humanas (ej: gestos, manipulación).

```python
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

class DemonstrationDataset(Dataset):
    """Dataset de demostraciones humanas"""
    def __init__(self, demonstrations):
        self.states = []
        self.actions = []
        
        for demo in demonstrations:
            self.states.extend(demo['states'])
            self.actions.extend(demo['actions'])
        
        self.states = torch.FloatTensor(self.states)
        self.actions = torch.FloatTensor(self.actions)
    
    def __len__(self):
        return len(self.states)
    
    def __getitem__(self, idx):
        return self.states[idx], self.actions[idx]

class BehavioralCloningNet(nn.Module):
    """Red para clonar comportamiento"""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()  # Acciones [-1, 1]
        )
    
    def forward(self, state):
        return self.net(state)

# Cargar demostraciones
demonstrations = load_demonstrations('demos/wave_gesture.pkl')

# Crear dataset
dataset = DemonstrationDataset(demonstrations)
dataloader = DataLoader(dataset, batch_size=64, shuffle=True)

# Crear modelo
model = BehavioralCloningNet(state_dim=40, action_dim=12)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=1e-3)

# Entrenar
for epoch in range(100):
    total_loss = 0
    for states, actions in dataloader:
        optimizer.zero_grad()
        
        pred_actions = model(states)
        loss = criterion(pred_actions, actions)
        
        loss.backward()
        optimizer.step()
        
        total_loss += loss.item()
    
    avg_loss = total_loss / len(dataloader)
    print(f"Epoch {epoch}: Loss = {avg_loss:.4f}")

# Guardar modelo
torch.save(model.state_dict(), 'models/wave_gesture_bc.pth')
```

## Optimización para Edge Deployment

**Cuantización de Modelos**:
Reducir tamaño y acelerar inferencia en Raspberry Pi / Jetson Nano.

```python
import torch
import torch.quantization

# Modelo entrenado
model = YourModel()
model.load_state_dict(torch.load('model.pth'))
model.eval()

# Cuantización dinámica (fácil, sin calibración)
quantized_model = torch.quantization.quantize_dynamic(
    model,
    {torch.nn.Linear},  # Capas a cuantizar
    dtype=torch.qint8
)

# Guardar modelo cuantizado
torch.save(quantized_model.state_dict(), 'model_quantized.pth')

# Comparar tamaño
import os
original_size = os.path.getsize('model.pth') / 1024 / 1024
quantized_size = os.path.getsize('model_quantized.pth') / 1024 / 1024

print(f"Original: {original_size:.2f} MB")
print(f"Quantized: {quantized_size:.2f} MB")
print(f"Reducción: {(1 - quantized_size/original_size)*100:.1f}%")

# Inferencia
with torch.no_grad():
    output = quantized_model(input_tensor)
```

**TensorRT (para Nvidia Jetson)**:
```python
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit

# Convertir modelo PyTorch a ONNX
torch.onnx.export(
    model,
    dummy_input,
    "model.onnx",
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}}
)

# Convertir ONNX a TensorRT (optimizado para Jetson)
# Usar herramienta trtexec o Python API
```

## Métricas y Evaluación

**Clasificación**:
- Accuracy, Precision, Recall, F1-Score
- Matriz de confusión

**Detección**:
- mAP (mean Average Precision)
- IoU (Intersection over Union)
- FPS (frames per second)

**RL**:
- Recompensa acumulada por episodio
- Tasa de éxito
- Número de pasos por episodio

**Ejemplo de evaluación**:
```python
from sklearn.metrics import accuracy_score, precision_recall_fscore_support, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns

def evaluate_classification(model, test_loader, class_names):
    """
    Evalúa modelo de clasificación
    """
    model.eval()
    
    all_preds = []
    all_labels = []
    
    with torch.no_grad():
        for images, labels in test_loader:
            outputs = model(images)
            _, preds = torch.max(outputs, 1)
            
            all_preds.extend(preds.cpu().numpy())
            all_labels.extend(labels.cpu().numpy())
    
    # Métricas
    accuracy = accuracy_score(all_labels, all_preds)
    precision, recall, f1, _ = precision_recall_fscore_support(
        all_labels, all_preds, average='weighted'
    )
    
    print(f"Accuracy: {accuracy:.4f}")
    print(f"Precision: {precision:.4f}")
    print(f"Recall: {recall:.4f}")
    print(f"F1-Score: {f1:.4f}")
    
    # Matriz de confusión
    cm = confusion_matrix(all_labels, all_preds)
    plt.figure(figsize=(10, 8))
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues',
                xticklabels=class_names, yticklabels=class_names)
    plt.xlabel('Predicted')
    plt.ylabel('True')
    plt.title('Confusion Matrix')
    plt.savefig('confusion_matrix.png')
    plt.show()

# Uso
evaluate_classification(model, test_loader, ['person', 'cup', 'book', ...])
```

## Datasets Recomendados

**Visión**:
- **COCO**: Detección de objetos (80 clases)
- **ImageNet**: Clasificación (1000 clases)
- **MPII**: Estimación de pose humana
- **VoxCeleb**: Reconocimiento facial

**Manipulación**:
- **RoboNet**: Datos de robots manipuladores
- **MIME**: Interacción móvil de manipulación

**Lenguaje**:
- **Common Voice**: Voz en múltiples idiomas
- **LibriSpeech**: Speech recognition en inglés

**Custom**:
Recolectar datos propios del robot en operación.

## Herramientas y Frameworks

**Deep Learning**:
- PyTorch, TensorFlow/Keras
- Hugging Face Transformers
- ONNX (interoperabilidad)

**RL**:
- Stable-Baselines3
- RLlib (Ray)
- OpenAI Gym

**Visión**:
- OpenCV
- Detectron2 (Facebook)
- MMDetection

**NLP**:
- spaCy, NLTK
- SpeechRecognition
- pyttsx3, gTTS

## Próximos Pasos

1. **Configurar entorno** de desarrollo (Python, PyTorch)
2. **Recolectar datos** iniciales (imágenes desde cámara)
3. **Entrenar modelo simple** (clasificación de objetos)
4. **Desplegar en robot** y probar inferencia
5. **Iterar**: Mejorar datos, modelo, optimización

## Referencias

- 📖 "Deep Learning" - Goodfellow, Bengio, Courville
- 📖 "Reinforcement Learning: An Introduction" - Sutton & Barto
- 🎓 Coursera: Deep Learning Specialization
- 🎓 Fast.ai: Practical Deep Learning
- 🌐 Papers With Code: State-of-the-art en AI

---

**Última actualización**: Febrero 2026