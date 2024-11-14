# Pallet Detection using YOLOv11n and ROS2

This repository contains the implementation of a pallet detection system utilizing YOLOv11n for object detection and semantic segmentation, integrated with ROS2 for real-time deployment. Follow the steps below to prepare, train, and deploy the system.

---

## **Table of Contents**

1. [Dataset Acquisition and Preparation](#dataset-acquisition-and-preparation)  
2. [Object Detection and Semantic Segmentation](#object-detection-and-semantic-segmentation)  
3. [ROS2 Node Development](#ros2-node-development)  
4. [How to Run](#how-to-run)

---

## **1. Dataset Acquisition and Preparation**

### Step 1: Download the Dataset
- Download your dataset from a reliable source or collect images for the application.  

### Step 2: Auto Label the Dataset  
- Use [Roboflow](https://app.roboflow.com/) and the Grounding DINO model to auto-label your dataset for object detection.  

### Step 3: Review Annotations  
- Carefully review the auto-annotated images. Make adjustments as needed to improve labeling accuracy and create the final dataset.  

### Step 4: Preprocess and Version the Dataset  
- Create a version of the dataset and include any preprocessing steps required.  
- Download the dataset version as necessary files for training.

### Step 5: Save in the Source Folder  
- Place the prepared dataset in the `src` folder for seamless integration with the training pipeline.

---

## **2. Object Detection and Semantic Segmentation**

### Step 1: Set Up Google Colab for Training  
Since local hardware lacks sufficient GPU resources, training is performed on Google Colab:  
- Install the required Python packages:  
  ```bash
  pip install ultralytics torch
  ```
- Use the `yolo11n-seg` model for training.

### Step 2: Train the YOLOv11n Model  
- Use the provided Colab notebook to train the model.  
- Follow the instructions in the notebook to ensure proper configuration of your training process.

### Step 3: Save the Trained Model  
- After training, download all trained model files from the Colab workspace.  
- Save the files to the `src` folder for deployment.

---

## **3. ROS2 Node Development**

### Step 1: Create a ROS2 Package  
- Create a ROS2 package named `pallet_detection` with required dependencies:  
  ```bash
  ros2 pkg create pallet_detection --build-type ament_python
  ```

### Step 2: Develop Publisher Nodes  
- Write two publisher nodes to publish image data on the `/camera/image_raw` topic:  
  - **Live Camera Data**: For real-world use.  
  - **Video File Data**: For testing using a pre-recorded video.  

> **Alternative**: You can also launch a Gazebo simulation with `turtlebot3 waffle` to get camera data.  

### Step 3: Write the Detection Node  
- Develop a subscriber node for the YOLOv11n segmentation model.  
  - This node subscribes to the `/camera/image_raw` topic.  
  - Ensure correct file paths for the trained model.

### Step 4: Update `setup.py`  
- Modify the `setup.py` file to include the newly created nodes.  

### Step 5: Build and Source the Workspace  
- Build the package:  
  ```bash
  colcon build
  ```
- Source your workspace:  
  ```bash
  source install/setup.bash
  ```

### Step 6: Run the Nodes  
- Launch the detection node with any of the following:  
  - Camera data node.  
  - Video file node (test video provided in the `pallet_detection` folder).  
  - Gazebo simulation.

---

## **4. How to Run**

### Step 1: Clone the Repository  
```bash
git clone https://github.com/vybhavraghav/Pallets-detection.git
cd Pallets-detection
```

### Step 2: Prepare the Dataset and Model  
- Place the prepared dataset and trained model files in the `src` folder.

### Step 3: Build and Source  
```bash
colcon build
source install/setup.bash
```

### Step 4: Launch the System  
Run the desired input node and detection node:  
```bash
ros2 run pallet_detection detection
```

In another terminal:
```bash
ros2 run pallet_detection video
(or)
ros2 run pallet_detection video
(or)
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py 
```


---


## **Acknowledgements**  
- [Roboflow](https://app.roboflow.com/) for dataset preparation.  
- Ultralytics for YOLOv11n.  
- ROS2 for real-time deployment.

Feel free to open issues or contribute to improve the project! 🚀
