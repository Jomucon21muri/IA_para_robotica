# 👁️ Visión por Computadora - Sistema Visual Robótico

## Propósito

Implementar el **sentido de la vista** del robot humanoide mediante cámaras y algoritmos de visión artificial. Este módulo es la base de la **Inteligencia Espacial-Visual** del robot, permitiéndole percibir, entender e interactuar con su entorno visual.

---

## 🌟 Capacidades del Sistema Visual

### Funciones Principales

| Capacidad | Descripción | Tecnología | Carpeta |
|-----------|-------------|-----------|----------|
| **Detección de objetos** | Identificar y localizar objetos en la escena | YOLO, Faster R-CNN | `detection/` |
| **Segmentación** | Separar objetos por píxeles (semántica/instancia) | Mask R-CNN, U-Net | `segmentation/` |
| **Reconocimiento facial** | Identificar y verificar personas | FaceNet, ArcFace | `face_recognition/` |
| **Estimación de pose** | Detectar posiciones de personas y objetos | OpenPose, MediaPipe | `pose_estimation/` |
| **SLAM Visual** | Mapeo y localización simultáneos | ORB-SLAM3, RTAB-Map | `slam/` |
| **Percepción 3D** | Reconstrucción tridimensional | Estéreo, RGB-D | `depth/` |
| **Seguimiento de objetos** | Rastrear objetos en movimiento | SORT, DeepSORT | `tracking/` |
| **Reconocimiento de gestos** | Interpretar gestos humanos | MediaPipe, CNN | `gestures/` |

---

## 📂 Estructura del Directorio

```
06_Vision/
├── README.md (este archivo)
├── detection/                      # Detección de objetos
│   ├── yolo/                      # YOLO v5/v8
│   ├── faster_rcnn/               # Faster R-CNN
│   ├── real_time/                 # Optimizaciones para tiempo real
│   └── custom_datasets/           # Datasets personalizados
├── segmentation/                   # Segmentación
│   ├── semantic/                  # Segmentación semántica
│   ├── instance/                  # Segmentación por instancia
│   └── panoptic/                  # Segmentación panóptica
├── face_recognition/               # Reconocimiento facial
│   ├── detection/                 # Detección de rostros
│   ├── recognition/               # Identificación
│   ├── emotion/                   # Detección de emociones
│   └── face_database/             # Base de datos de rostros
├── pose_estimation/                # Estimación de pose
│   ├── human_pose/                # Pose humana 2D/3D
│   ├── hand_tracking/             # Seguimiento de manos
│   └── body_tracking/             # Tracking corporal completo
├── slam/                          # SLAM Visual
│   ├── orb_slam/                  # ORB-SLAM3
│   ├── rtabmap/                   # RTAB-Map
│   └── visual_odometry/           # Odometría visual
├── depth/                         # Percepción de profundidad
│   ├── stereo_vision/             # Visión estéreo
│   ├── rgbd/                      # RGB-D (RealSense, Kinect)
│   └── depth_estimation/          # Estimación monocular
├── tracking/                      # Seguimiento de objetos
│   ├── sort/                      # SORT algorithm
│   ├── deepsort/                  # DeepSORT
│   └── multi_object/              # Tracking multi-objeto
├── gestures/                      # Reconocimiento de gestos
│   ├── hand_gestures/             # Gestos de manos
│   ├── body_gestures/             # Gestos corporales
│   └── gesture_commands/          # Comandos por gestos
├── ocr/                           # Reconocimiento de texto
│   ├── text_detection/            # Detección de texto
│   ├── text_recognition/          # OCR (Tesseract, EasyOCR)
│   └── scene_text/                # Texto en escenas naturales
├── preprocessing/                  # Preprocesamiento
│   ├── calibration/               # Calibración de cámaras
│   ├── enhancement/               # Mejora de imagen
│   └── augmentation/              # Data augmentation
├── models/                        # Modelos entrenados
│   ├── pretrained/                # Modelos preentrenados
│   └── finetuned/                 # Modelos ajustados
├── datasets/                      # Datasets
│   ├── coco/                      # COCO dataset
│   ├── kitti/                     # KITTI (conducción)
│   ├── custom/                    # Datos propios
│   └── scripts/                   # Scripts de conversión
└── notebooks/                     # Notebooks de demostración
    ├── object_detection_demo.ipynb
    ├── face_recognition_demo.ipynb
    ├── slam_visualization.ipynb
    └── depth_estimation_demo.ipynb
```

---

## 1. 👁️ Detección de Objetos

### Implementación con YOLO v8

```python
from ultralytics import YOLO
import cv2
import numpy as np

class ObjectDetector:
    def __init__(self, model_path='yolov8n.pt', conf_threshold=0.5):
        """
        Inicializa detector de objetos
        model_path: yolov8n (nano), yolov8s (small), yolov8m (medium), yolov8l (large)
        """
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.class_names = self.model.names
        
    def detect(self, frame):
        """Detecta objetos en un frame"""
        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        
        detections = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Extraer información
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                class_name = self.class_names[cls]
                
                detections.append({
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'confidence': conf,
                    'class': class_name,
                    'class_id': cls
                })
        
        return detections
    
    def draw_detections(self, frame, detections):
        """Dibuja las detecciones en el frame"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            conf = det['confidence']
            label = f"{det['class']} {conf:.2f}"
            
            # Dibujar bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Dibujar etiqueta
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(frame, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)
            cv2.putText(frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        return frame

# Uso en tiempo real
detector = ObjectDetector('yolov8n.pt')
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    detections = detector.detect(frame)
    frame = detector.draw_detections(frame, detections)
    
    cv2.imshow('Object Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### Integración con ROS

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2

class YOLODetectorNode:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        
        self.bridge = CvBridge()
        self.detector = ObjectDetector('yolov8n.pt')
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Publishers
        self.detection_pub = rospy.Publisher('/vision/detections', Detection2DArray, queue_size=10)
        self.viz_pub = rospy.Publisher('/vision/detection_viz', Image, queue_size=10)
        
    def image_callback(self, msg):
        # Convertir ROS Image a OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Detectar objetos
        detections = self.detector.detect(cv_image)
        
        # Publicar detecciones
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header
        
        for det in detections:
            detection_2d = Detection2D()
            detection_2d.bbox.center.x = (det['bbox'][0] + det['bbox'][2]) / 2
            detection_2d.bbox.center.y = (det['bbox'][1] + det['bbox'][3]) / 2
            detection_2d.bbox.size_x = det['bbox'][2] - det['bbox'][0]
            detection_2d.bbox.size_y = det['bbox'][3] - det['bbox'][1]
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = det['class_id']
            hypothesis.score = det['confidence']
            detection_2d.results.append(hypothesis)
            
            detection_msg.detections.append(detection_2d)
        
        self.detection_pub.publish(detection_msg)
        
        # Publicar visualización
        viz_image = self.detector.draw_detections(cv_image.copy(), detections)
        viz_msg = self.bridge.cv2_to_imgmsg(viz_image, "bgr8")
        self.viz_pub.publish(viz_msg)

if __name__ == '__main__':
    try:
        node = YOLODetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

---

## 2. 🎭 Reconocimiento Facial y Emociones

### Detección y reconocimiento de rostros

```python
import face_recognition
import cv2
import numpy as np

class FaceRecognitionSystem:
    def __init__(self):
        self.known_face_encodings = []
        self.known_face_names = []
        
    def add_person(self, image_path, name):
        """Añade una persona a la base de datos"""
        image = face_recognition.load_image_file(image_path)
        encodings = face_recognition.face_encodings(image)
        
        if len(encodings) > 0:
            self.known_face_encodings.append(encodings[0])
            self.known_face_names.append(name)
            return True
        return False
    
    def recognize_faces(self, frame):
        """Reconoce rostros en un frame"""
        # Reducir tamaño para procesamiento más rápido
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        
        # Encontrar rostros
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        
        face_names = []
        for face_encoding in face_encodings:
            # Comparar con rostros conocidos
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Desconocido"
            
            # Usar el rostro con menor distancia
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            if len(face_distances) > 0:
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = self.known_face_names[best_match_index]
            
            face_names.append(name)
        
        # Escalar de vuelta las ubicaciones
        face_locations = [(top*4, right*4, bottom*4, left*4) 
                         for (top, right, bottom, left) in face_locations]
        
        return face_locations, face_names
```

### Detección de emociones

```python
from deepface import DeepFace

class EmotionDetector:
    def __init__(self):
        self.emotions = ['angry', 'disgust', 'fear', 'happy', 'sad', 'surprise', 'neutral']
    
    def detect_emotions(self, frame):
        """Detecta emociones en rostros"""
        try:
            result = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)
            
            if isinstance(result, list):
                emotions_detected = []
                for face in result:
                    emotions_detected.append({
                        'region': face['region'],
                        'dominant_emotion': face['dominant_emotion'],
                        'emotion_scores': face['emotion']
                    })
                return emotions_detected
            else:
                return [{
                    'region': result['region'],
                    'dominant_emotion': result['dominant_emotion'],
                    'emotion_scores': result['emotion']
                }]
        except:
            return []
```

---

## 3. 🗺️ SLAM Visual

### Implementación con ORB-SLAM3 (ROS)

```bash
# Instalación
cd ~/catkin_ws/src
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
./build.sh

# Lanzar SLAM con cámara monocular
rosrun ORB_SLAM3 Mono vocabulary.txt camera_config.yaml
```

### RTAB-Map (RGB-D SLAM)

```bash
# Lanzar RTAB-Map con RealSense
roslaunch realsense2_camera rs_camera.launch

roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/depth/image_rect_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false
```

---

## 4. 📐 Percepción de Profundidad

### Visión estéreo

```python
import cv2
import numpy as np

class StereoDepthEstimator:
    def __init__(self, baseline=0.06, focal_length=700):
        """
        baseline: distancia entre cámaras en metros
        focal_length: focal length en píxeles
        """
        self.baseline = baseline
        self.focal_length = focal_length
        
        # Crear matcher estéreo
        self.stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)
        
    def compute_disparity(self, img_left, img_right):
        """Calcula mapa de disparidad"""
        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        
        disparity = self.stereo.compute(gray_left, gray_right)
        return disparity
    
    def disparity_to_depth(self, disparity):
        """Convierte disparidad a profundidad (en metros)"""
        # Evitar división por cero
        disparity = disparity.astype(np.float32) / 16.0
        disparity[disparity == 0] = 0.1
        
        # Z = (baseline * focal_length) / disparity
        depth = (self.baseline * self.focal_length) / disparity
        return depth
    
    def get_point_cloud(self, disparity, img_left):
        """Genera nube de puntos 3D"""
        depth = self.disparity_to_depth(disparity)
        
        height, width = disparity.shape
        points = []
        colors = []
        
        for v in range(height):
            for u in range(width):
                if depth[v, u] > 0 and depth[v, u] < 10:  # Filtrar ruido
                    z = depth[v, u]
                    x = (u - width/2) * z / self.focal_length
                    y = (v - height/2) * z / self.focal_length
                    
                    points.append([x, y, z])
                    colors.append(img_left[v, u] / 255.0)
        
        return np.array(points), np.array(colors)
```

### Usando RealSense RGB-D

```python
import pyrealsense2 as rs
import numpy as np

class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        self.pipeline.start(config)
        
        # Align depth to color
        self.align = rs.align(rs.stream.color)
        
    def get_frames(self):
        """Obtiene frames RGB y Depth alineados"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return None, None
        
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        return color_image, depth_image
    
    def get_3d_point(self, x, y, depth_frame):
        """Obtiene coordenadas 3D de un píxel"""
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        depth = depth_frame.get_distance(x, y)
        point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
        return point_3d
    
    def stop(self):
        self.pipeline.stop()
```

---

## 5. 🤚 Estimación de Pose y Gestos

### Detección de pose humana con MediaPipe

```python
import mediapipe as mp
import cv2

class PoseEstimator:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
    
    def process_frame(self, frame):
        """Procesa frame y detecta pose"""
        # Convertir a RGB
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)
        
        if results.pose_landmarks:
            # Dibujar landmarks
            self.mp_drawing.draw_landmarks(
                frame,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS
            )
            
            # Extraer coordenadas de puntos clave
            landmarks = results.pose_landmarks.landmark
            keypoints = []
            for lm in landmarks:
                keypoints.append({
                    'x': lm.x,
                    'y': lm.y,
                    'z': lm.z,
                    'visibility': lm.visibility
                })
            
            return frame, keypoints
        
        return frame, None
    
    def detect_gesture(self, keypoints):
        """Detecta gestos específicos basados en pose"""
        if not keypoints:
            return "none"
        
        # Ejemplo: detectar brazos levantados
        left_shoulder = keypoints[11]
        left_wrist = keypoints[15]
        right_shoulder = keypoints[12]
        right_wrist = keypoints[16]
        
        if left_wrist['y'] < left_shoulder['y'] and right_wrist['y'] < right_shoulder['y']:
            return "both_arms_up"
        elif left_wrist['y'] < left_shoulder['y']:
            return "left_arm_up"
        elif right_wrist['y'] < right_shoulder['y']:
            return "right_arm_up"
        
        return "neutral"
```

---

## 📊 Pipeline de Visión Completo

```python
class VisionPipeline:
    """Pipeline completo de visión para el robot"""
    
    def __init__(self):
        self.object_detector = ObjectDetector('yolov8n.pt')
        self.face_recognizer = FaceRecognitionSystem()
        self.emotion_detector = EmotionDetector()
        self.pose_estimator = PoseEstimator()
        self.depth_camera = RealSenseCamera()
        
    def process_frame(self, mode='full'):
        """
        Procesa frame según el modo
        mode: 'detection', 'faces', 'pose', 'full'
        """
        color_frame, depth_frame = self.depth_camera.get_frames()
        
        if color_frame is None:
            return None
        
        results = {
            'timestamp': time.time(),
            'objects': [],
            'faces': [],
            'emotions': [],
            'pose': None,
            'gesture': None
        }
        
        if mode in ['detection', 'full']:
            results['objects'] = self.object_detector.detect(color_frame)
        
        if mode in ['faces', 'full']:
            face_locations, face_names = self.face_recognizer.recognize_faces(color_frame)
            results['faces'] = list(zip(face_locations, face_names))
            results['emotions'] = self.emotion_detector.detect_emotions(color_frame)
        
        if mode in ['pose', 'full']:
            _, keypoints = self.pose_estimator.process_frame(color_frame)
            results['pose'] = keypoints
            if keypoints:
                results['gesture'] = self.pose_estimator.detect_gesture(keypoints)
        
        return results, color_frame, depth_frame
```

---

## 🔧 Calibración de Cámaras

```python
import cv2
import numpy as np
import glob

def calibrate_camera(images_path, pattern_size=(9, 6), square_size=0.025):
    """
    Calibra cámara usando patrón de ajedrez
    pattern_size: (columnas, filas) de esquinas internas
    square_size: tamaño del cuadrado en metros
    """
    # Preparar puntos del patrón 3D
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    # Arrays para almacenar puntos
    objpoints = []  # Puntos 3D en el mundo real
    imgpoints = []  # Puntos 2D en la imagen
    
    images = glob.glob(images_path + '/*.jpg')
    
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Encontrar esquinas del tablero
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
    
    # Calibrar
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    
    return mtx, dist, rvecs, tvecs

# Guardar parámetros de calibración
def save_calibration(filename, mtx, dist):
    np.savez(filename, mtx=mtx, dist=dist)

# Cargar parámetros
def load_calibration(filename):
    data = np.load(filename)
    return data['mtx'], data['dist']
```

---

## 📚 Referencias y Recursos

### Bibliotecas

```bash
# Instalar dependencias
pip install opencv-python opencv-contrib-python
pip install ultralytics  # YOLO
pip install face-recognition
pip install deepface
pip install mediapipe
pip install pyrealsense2
pip install torch torchvision
```

### Datasets recomendados
- **COCO**: Common Objects in Context (detección de objetos)
- **KITTI**: Autonomous driving dataset
- **ImageNet**: Clasificación de imágenes
- **WIDER FACE**: Detección de rostros
- **LFW**: Labeled Faces in the Wild
- **MPII**: Human pose estimation

### Papers importantes
1. "You Only Look Once: Unified, Real-Time Object Detection" (Redmon et al., 2016)
2. "Mask R-CNN" (He et al., 2017)
3. "ORB-SLAM3" (Campos et al., 2021)
4. "FaceNet" (Schroff et al., 2015)

---

**Ver también**:
- [🧠 Percepción](../01_Percepcion/) - Fusión sensorial multimodal
- [📍 Localización y Mapeo](../02_Localizacion_Mapeo/) - SLAM
- [🤖 Aprendizaje de Máquina](../05_Aprendizaje_Maquina/) - Entrenamiento de modelos
- [📚 Recursos de Conocimiento](../00_Gestion_Proyecto/recursos_conocimientos.md)