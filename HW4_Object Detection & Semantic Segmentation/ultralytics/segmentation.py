from ultralytics import YOLO

def train():
  model = YOLO("yolo11n-seg.pt")  # load a pretrained model (recommended for training)
  
  # Train the model
  results = model.train(
    data="../dataset_label/Pros_car_segmentation.v1i.yolov11/data.yaml",
    epochs=200,
    imgsz=640,
    batch=16,
    device="cpu",   
    workers=4,      
    save=True,
    name="floor_seg",  
)

def inference():
  model = YOLO("./runs/segment/floor_seg/weights/best.pt")
  results = model(
    source="../dataset_label/Pros_car_segmentation.v1i.yolov11/test/images",
    save=True, 
    imgsz=640, 
    conf=0.5)

if __name__ == "__main__":
  train()
  inference()
