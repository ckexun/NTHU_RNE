# NTHU_RNE

**Robotic Navigation and Exploration (RNE)** course projects at NTHU.  
Includes path planning, path tracking control, deep reinforcement learning, and computer vision with YOLO, with environment setup and execution supported by Docker.


---

## Environment
[![Python](https://img.shields.io/badge/Python-3.6%2B-blue.svg)](https://www.python.org/)  [![PyTorch](https://img.shields.io/badge/PyTorch-Deep%20Learning-red.svg)](https://pytorch.org/)  
[![Docker](https://img.shields.io/badge/Docker-Enabled-blue.svg)](https://www.docker.com/) [![Ultralytics YOLO](https://img.shields.io/badge/YOLO-Ultralytics-green.svg)](https://docs.ultralytics.com/)  

---

## Labs Overview

### Path Planning
- **A\***: Heuristic search algorithm  
    - Implement path planning using A* algorithm
- **RRT / RRT\***: Rapidly-exploring Random Trees and optimized version  
    - Implement path planning using RRT* algorithm
- **Focus**: Implementation, correctness, efficiency  


### Path Tracking Control
- **Controllers**: PID, Pure Pursuit, Stanley, LQR  
- **Vehicle Models**: Basic, Differential Drive, Bicycle  
- **Extra**: Collision handling & re-planning  

### Deep Reinforcement Learning on Path Tracking
- **Algorithm**: Proximal Policy Optimization (PPO) 
- **Implementation**: Policy / Value networks, environment runner, training & evaluation  


### Object Detection & Semantic Segmentation
- **YOLOv11 (Ultralytics)** for object detection and segmentation  
- **Data Collection**: Images captured via PROS Twin system with annotation  
- **Annotation Tool**: Used **Roboflow** for image labeling (Bounding Box and Polygon Segmentation)  
- **Tasks**:  
  - Train YOLOv11n for Pikachu object detection('Kirito', 'chair', 'pikachu', 'sofa', 'table')  
  - Train YOLOv11n for scene semantic segmentation ('carpet', 'floor', 'floor mat')  
- **Deliverables**: Two trained models(ultralytics/runs/*)

---


## Execution

```bash
# Path Planning
python path_planning.py -p [a_star/rrt/rrt_star]

# Path Tracking
python navigation.py -s [basic/diff_drive/bicycle] -c [pid/pure_pursuit/stanley/lqr] -p [a_star/rrt/rrt_star]

# Reinforcement Learning
python train.py
python play.py
# python plot.py
python eval.py

# YOLO Training
python object_detection.py
python segmentation.py

# YOLO Testing (ROS2 Integration)
ros2 run yolo_pkg yolo_detection_node
```

---

## Result
### Path Planning
- **A\***  
  ![A*](HW1_Path%20Planning/code_practice/result/a_star.png)

- **RRT**  
  ![RRT](HW1_Path%20Planning/code_practice/result/rrt.gif)

- **RRT\***  
  ![RRT*](HW1_Path%20Planning/code_practice/result/rrt_star.gif)
### Path Tracking Control
- **Basic PID + A\***  
  ![Basic PID + A*](HW2_Path%20Tracking/code_practice/result/basic_pid_a_star.png)

- **Bicycle Stanley + RRT\***  
  ![Bicycle Stanley + RRT*](HW2_Path%20Tracking/code_practice/result/bicycle_stanley_rrt_star.png)

- **Differential Drive Pure Pursuit + RRT**  
  ![Diff Drive Pure Pursuit + RRT](HW2_Path%20Tracking/code_practice/result/diff_drive_pure_pursuit_rrt.png)


### Deep Reinforcement Learning on Path Tracking
- **PPO Training Curve**  
  ![PPO](HW3_Deep%20Reinforcement%20Learning%20on%20Path%20Tracking/code_practice/result/PPO.png)
### Object Detection & Semantic Segmentation
#### 🔍 Object Detection
- **Confusion Matrix**  
  ![Confusion Matrix (Detection)](HW4_Object%20Detection%20%26%20Semantic%20Segmentation/result/detect/confusion_matrix_normalized.png)

- **F1 Curve**  
  ![F1 Curve (Detection)](HW4_Object%20Detection%20%26%20Semantic%20Segmentation/result/detect/F1_curve.png)

- **Validation Prediction**  
  ![Validation Example (Detection)](HW4_Object%20Detection%20%26%20Semantic%20Segmentation/result/detect/val_batch1_pred.jpg)


#### Semantic Segmentation
- **Confusion Matrix**  
  ![Confusion Matrix (Segmentation)](HW4_Object%20Detection%20%26%20Semantic%20Segmentation/result/segment/confusion_matrix_normalized.png)

- **F1 Curve**  
  ![F1 Curve (Segmentation)](HW4_Object%20Detection%20%26%20Semantic%20Segmentation/result/segment/BoxF1_curve.png)

- **Validation Prediction**  
  ![Validation Example (Segmentation)](HW4_Object%20Detection%20%26%20Semantic%20Segmentation/result/segment/val_batch0_pred.jpg)