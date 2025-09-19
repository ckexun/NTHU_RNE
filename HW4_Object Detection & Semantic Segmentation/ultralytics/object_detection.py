from ultralytics import YOLO

def train():
  # Load a COCO-pretrained YOLO11n model
  model = YOLO("yolo11n.pt")
  
  # Train the model on the COCO8 example dataset for 100 epochs
  # results = model.train(data="../dataset_label/Pros_car_obeject_detection.v1i.yolov11/data.yaml", epochs=200, imgsz=640)
  results = model.train(
    data="../dataset_label/Pros_car_obeject_detection.v1i.yolov11/data.yaml",
    epochs=200,
    imgsz=640,
    batch=16,
    device="cpu",   
    workers=4,       
    save=True,
    name="pikachu_det",  
)

def inference():
  # Run inference with the YOLO11n model on the 'bus.jpg' image
  # results = model("inference/bus.jpg")
  model = YOLO("./runs/detect/pikachu_det3/weights/best.pt")
  results = model(
    source="../dataset_label/Pros_car_obeject_detection.v1i.yolov11/test/images",
    save=True, 
    imgsz=640, 
    conf=0.5)

if __name__ == "__main__":
  # train()
  inference()

