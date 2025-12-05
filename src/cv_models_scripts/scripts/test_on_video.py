from ultralytics import YOLO
from rfdetr import RFDETRNano
import cv2
import torch
import supervision as sv

CONF_THRESHOLD = 0.25
BOX_ANNOTATOR = sv.BoxAnnotator()
LABEL_ANNOTATOR = sv.LabelAnnotator()
MODEL = RFDETRNano(pretrain_weights='../runs/RF_DETR_nano_train/checkpoint_best_total.pth')
CLASSES = ['blue_armor', 'grey_armor', 'purple_armor', 'red_armor']

def callback(frame, index):
    """
    Processes a single video frame with the RF-DETR model.
    This function replaces the body of your manual 'while cap.isOpened()' loop.
    """
    
    # 1. Predict on the single frame
    # NOTE: RF-DETR expects RGB, so frame[:, :, ::-1] converts BGR (OpenCV default) to RGB.
    # The output 'detections' here is the raw RF-DETR output dictionary.
    detections = MODEL.predict(frame[:, :, ::-1].copy(), threshold=CONF_THRESHOLD) 

    # 3. Prepare Labels (Class Name and Confidence)
    labels = [
        f"{CLASSES[class_id]} {confidence:.2f}"
        for class_id, confidence 
        in zip(detections.class_id, detections.confidence)
    ]

    # 4. Annotate the frame
    annotated_frame = frame.copy()
    
    annotated_frame = BOX_ANNOTATOR.annotate(annotated_frame, detections)
    annotated_frame = LABEL_ANNOTATOR.annotate(annotated_frame, detections, labels=labels)

    # print('Frame annotated')
    sv.plot_image(annotated_frame)

    return annotated_frame


def main():
    model = YOLO('../runs/yolov8n_train2/weights/best.pt')
    #model = RFDETRNano(pretrain_weights='../runs/RF_DETR_nano_train/checkpoint_best_total.pth')
    #MODEL.optimize_for_inference()

    # sv.process_video(
    #     source_path='BeaminSlowmo2.mp4', 
    #     target_path='annotated_BeaminSlowmo2_RFDETR.mp4',
    #     callback=callback
    # )

    model.predict(
        source='VT.mp4',
        device='cuda' if torch.cuda.is_available() else 'cpu',
        show=True,
        save=True,
        rect=True,
        conf=0.25,
        iou=0.7,
        verbose=False
    )

if __name__ == '__main__':
    main()