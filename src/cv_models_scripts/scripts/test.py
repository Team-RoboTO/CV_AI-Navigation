from ultralytics import YOLO
import torch
import cv2
import numpy as np
import os
import shutil
import tensorrt as trt
from train import save_config_to_yaml
from rfdetr import RFDETRNano


class TestConfig:
    dataset_path = "../dataset"
    test_dir = f"{dataset_path}/test/images"
    test_labels_dir = f"{dataset_path}/test/labels"
    temp_test_dir = "../test_aug"
    test_runs_dir = "../test_runs"
    runs_dir = "../runs"
    model_type = "RF_DETR"
    model_weights = f"{runs_dir}/{model_type}_train/checkpoint_best_total.pth"
    train_config_path = f"{runs_dir}/{model_type}_train/train_config.yaml"
    imgsz = 320
    confidence_thresh = 0.25
    max_brightness_percentage = 0.4
    max_zoom_percentage = 0.3
    device = 'cuda' if torch.cuda.is_available() else 'cpu'


def change_brightness(img, factor):
    """
    Changes the brightness of an input image **img** by multiplying the value channel in its hsv representation by **factor**.

    **Input**:
        - img: image in BGR color format
        - factor: brightness factor

    **Output**:
        - brightness-modified image (original hsv value * *factor*) in BGR color format
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # avoid uint8 overflow during multiplication
    hsv = hsv.astype(np.float32)
    # amplify/reduce hsv value (i.e.: brightness level) by factor
    hsv[:, :, 2] = hsv[:, :, 2] * factor
     # ensure to have valid hsv values after applying the brightness factor
    hsv[:, :, 2] = np.clip(hsv[:, :, 2], 0, 255)
    hsv = hsv.astype(np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def change_zoom(img, factor):
    """
    Zooms in or out of an input image **img** by scaling it by **factor** and cropping/padding to the original size.

    **Input**:
        - img: image in BGR color format (img)
        - factor: zoom factor (if > 1 -> zoom in, if < 1 -> zoom out)

    **Output**:
        - zoom-modified image in BGR color format, keeping the original dimensions
    """
    h, w = img.shape[:2]
    
    # compute the new dimensions after scaling
    new_h, new_w = int(h * factor), int(w * factor)
    
    # resize the image
    scaled_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # compute the offset for centering the image
    offset_h = abs(new_h - h) // 2
    offset_w = abs(new_w - w) // 2

    out = np.zeros((h, w, 3), dtype=img.dtype)

    if factor >= 1.0:
        # zoom in (Factor > 1.0): crop the center
        out = scaled_img[offset_h : offset_h + h, offset_w : offset_w + w]
    else:
        # zoom out (Factor < 1.0): pad the center area
        start_h = offset_h
        start_w = offset_w
        
        # compute the bottom-right corner (end index)
        end_h = start_h + new_h
        end_w = start_w + new_w

        # place the scaled image onto the center of the black canvas (out)
        out[start_h:end_h, start_w:end_w] = scaled_img
        
    return out


def robustify_test_images(test_dir, save_dir, config: TestConfig):
    """
    Given a directory of test images, generates a new directory containing a modified version of them such that:
        - their brightness shifted by a random factor
        - they are zoomed in/out by another random factor

    **Input**:
        - test_dir: source directory of test images
        - save_dir: path to the directory where the modified images will be saved
        - config: testing configuration instance
    """
    for file_name in os.listdir(test_dir):
        if file_name.lower().endswith((".jpg", ".png", ".jpeg")):
            img_path = os.path.join(test_dir, file_name)
            img = cv2.imread(img_path)

            br_factor = np.random.uniform(1 - config.max_brightness_percentage, 1 + config.max_brightness_percentage) # -40% to +40%
            out = change_brightness(img, br_factor)

            z_factor = np.random.uniform(1 - config.max_zoom_percentage, 1 + config.max_zoom_percentage) # -30% to +30%
            out = change_zoom(out, z_factor)

            cv2.imwrite(os.path.join(save_dir, file_name), cv2.cvtColor(out, cv2.COLOR_BGR2RGB))


def export_to_onnx(config: TestConfig):
    """
    Exports a trained YOLO PyTorch model to the **ONNX** (***Open Neural Network Exchange***) format.
    The exported ONNX file will be saved in the same directory as the input .pt file.

    **Input**:
        - config: testing configuration instance
    """

    if "yolo" in config.model_type:
        model = YOLO(config.model_weights)
    else:
        model = RFDETRNano(pretrain_weights=config.model_weights)

    output_dir = os.path.dirname(config.model_weights)
    
    model.export(
        format="onnx",
        output_dir=output_dir,
        opset=12,      # ONNX Operator Set version (for modern models a common value is >= 12)
        imgsz=config.imgsz,   # input image size for the resulting ONNX graph
        dynamic=False, # set this to False when using a fixed batch size and image size
        simplify=True, # set this to True to optimize the model structure for faster inference
        half=True      # export using Half Precision (FP16) arithmetic to reduce model size and to speed up inference
    )


def generate_plan_file(onnx_file_path, engine_file_path, fp16=True):
    """
    Compiles an ONNX model into a serialized TensorRT engine file (.plan) 
    optimized for NVIDIA GPUs. It sets up the TensorRT builder, parses the ONNX graph, 
    and optionally enables FP16 precision before building the optimized engine.

    **Input**:
        - config: testing configuration instance
    """
    # setup the Logger
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    
    # create the Builder and Network
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, TRT_LOGGER)
    config = builder.create_builder_config()

    # enable FP16 precision (recommended for fast GPU inference)
    if fp16:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("Enabling FP16 precision")
        else:
            print("FP16 not supported on this platform, using FP32")

    # parse the ONNX File
    print(f"Loading ONNX file from: {onnx_file_path}")
    if not os.path.exists(onnx_file_path):
        print(f"Error: File {onnx_file_path} not found.")
        return

    with open(onnx_file_path, 'rb') as model:
        if not parser.parse(model.read()):
            print("ERROR: Failed to parse the ONNX file.")
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            return None

    # build the engine
    print("Building TensorRT Engine...")
    serialized_engine = builder.build_serialized_network(network, config)

    if serialized_engine:
        print(f"Saving Plan file to: {engine_file_path}")
        with open(engine_file_path, "wb") as f:
            f.write(serialized_engine)
        print("Success!")
    else:
        print("Failed to build engine.")


def compute_metrics(results, model_weights, test_aug_dir, labels_dir, output_file, train_config_path):
    """
    Computes and saves evaluation metrics from YOLO prediction results.

    **Input**:
        - results: YOLO prediction results object
        - model_weights: path to the model weights used for prediction
        - test_aug_dir: path to the directory containing the augmented test images
        - labels_dir: path to the directory containing the ground truth labels for the test images
        - output_file: path to the output file where metrics will be saved
        - train_config_path: path to the training config YAML file related to the results considered
    """
    num_images = len(results)
    total_detections = 0
    total_confidence = 0
    total_latency = 0
    
    # collect detections, confidences, and latency
    for result in results:
        if result.boxes is not None and len(result.boxes) > 0:
            total_detections += len(result.boxes)
            total_confidence += result.boxes.conf.sum().item()
        
        if result.speed is not None:
            total_latency += sum(result.speed.values())
    
    # compute metrics
    avg_confidence = total_confidence / total_detections if total_detections > 0 else 0
    avg_total_latency = total_latency / num_images if num_images > 0 else 0  # ms per image
    fps = 1000 / avg_total_latency if avg_total_latency > 0 else 0  # frames per second

    test_run_dir = os.path.dirname(output_file)
    
    # compute mAP50 on test_aug
    map50 = -1
    try:
        # create proper YOLO directory structure in test_aug
        temp_images_dir = os.path.join(test_aug_dir, "images")
        temp_labels_dir = os.path.join(test_aug_dir, "labels")
        
        # move images to images subfolder
        if not os.path.exists(temp_images_dir):
            os.makedirs(temp_images_dir)
            for img_file in os.listdir(test_aug_dir):
                if img_file.lower().endswith((".jpg", ".png", ".jpeg")):
                    shutil.move(
                        os.path.join(test_aug_dir, img_file),
                        os.path.join(temp_images_dir, img_file)
                    )
        
        # copy labels to labels subfolder
        if os.path.exists(temp_labels_dir):
            shutil.rmtree(temp_labels_dir)
        shutil.copytree(labels_dir, temp_labels_dir)
        
        # create temporary YAML to extract map50 directly from the test split
        temp_yaml = os.path.join(os.path.dirname(test_aug_dir), "temp_test.yaml")
        
        with open(temp_yaml, 'w') as f:
            f.write("train:\n")
            f.write("val:\n")
            f.write(f"test: {test_aug_dir}/images\n")
            f.write(f"nc: {len(results[0].names)}\n")
            f.write(f"names: {list(results[0].names.values())}\n")
        
        model = YOLO(model_weights)
        val_dir = f"{os.path.basename(test_run_dir)}_val"
        val_results = model.val(data=temp_yaml, split='test', project=os.path.dirname(test_run_dir), name=val_dir)
        map50 = val_results.box.map50
        
        # cleanup (temporary YAML and the "runs" folder created after calling val())
        os.remove(temp_yaml)
        
        temp_val_path = os.path.join(os.path.dirname(test_run_dir), val_dir)
        
        if os.path.exists(temp_val_path):
            
            for item_name in os.listdir(temp_val_path):
                src = os.path.join(temp_val_path, item_name)
                dst = os.path.join(test_run_dir, item_name)
                
                if os.path.exists(dst):
                    if os.path.isfile(dst):
                        os.remove(dst)
                    elif os.path.isdir(dst):
                        shutil.rmtree(dst)
                        
                shutil.move(src, dst)
            
            #shutil.rmtree(temp_val_path)
        #shutil.rmtree("./runs")
    except Exception as e:
        print(f"Warning: could not compute mAP50: {e}")
        pass
    
    # save metrics
    with open(output_file, 'w') as f:
        f.write(f"mAP50: {map50:.4f}\n")
        f.write(f"FPS: {fps:.2f}\n")
        f.write(f"Avg confidence: {avg_confidence:.4f}\n")
        f.write(f"Latency: {avg_total_latency:.2f} ms/image\n")
        f.write(f"Training config path: {train_config_path}\n")


def main():

    config = TestConfig()

    os.makedirs(config.temp_test_dir, exist_ok=True)
    yolo_model = YOLO(config.model_weights)
    
    robustify_test_images(test_dir=config.test_dir, save_dir=config.temp_test_dir, config=config)

    run_name = f"{config.model_type}_test"
    project_dir = config.test_runs_dir

    res = yolo_model.predict(
        source=config.temp_test_dir,
        conf=config.confidence_thresh,
        imgsz=config.imgsz,
        device=config.device,
        rect=True,
        half=True,
        max_det=300,
        show=False,
        save=True,
        save_txt=True,
        save_conf=True,
        show_labels=True,
        show_conf=True,
        show_boxes=True,
        name=run_name,
        project=project_dir
    )

    # avoid AttributeError in case the used YOLO version does not provide this attribute
    if hasattr(res[0], 'save_dir') and res[0].save_dir:
        actual_run_dir = res[0].save_dir
    else:
        actual_run_dir = os.path.join(project_dir, run_name)

    metrics_file = os.path.join(actual_run_dir, "metrics.txt")
    save_config_to_yaml(config_instance=config, config_save_dir=actual_run_dir)

    compute_metrics(
        results=res,
        model_weights=config.model_weights,
        test_aug_dir=config.temp_test_dir,
        labels_dir=config.test_labels_dir,
        output_file=metrics_file,
        train_config_path=config.train_config_path
    )

    os.makedirs(os.path.join(actual_run_dir, "images"), exist_ok=True)

    for file_name in os.listdir(actual_run_dir):

        if file_name.lower().endswith((".jpg", ".jpeg", ".png")):
            src_path = os.path.join(actual_run_dir, file_name)
            dst_path = os.path.join(os.path.join(actual_run_dir, "images"), file_name)
            shutil.move(src_path, dst_path)

    shutil.rmtree(config.temp_test_dir)


if __name__ == "__main__":
    #main()
    config = TestConfig()
    #export_to_onnx(config=config)
    model_dir = os.path.dirname(config.model_weights)
    onnx_path = os.path.join(model_dir, "inference_model.onnx")
    engine_path = os.path.join(model_dir, "inference_model.sim.onnx")
    generate_plan_file(onnx_file_path=onnx_path, engine_file_path=engine_path, fp16=True)
