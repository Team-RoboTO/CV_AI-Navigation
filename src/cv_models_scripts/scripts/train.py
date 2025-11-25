from ultralytics import YOLO
import torch
import os
import json
import yaml


class TrainConfig:
    dataset_path = "../dataset"
    data_yaml = f"{dataset_path}/data.yaml" 
    model_name = "yolo11n.pt"
    runs_dir = "../runs"
    imgsz = 320
    num_epochs = 100
    batch_size = 10
    lr0 = 0.01
    weight_decay = 0.0005
    patience = num_epochs // 5
    device = 'cuda' if torch.cuda.is_available() else 'cpu'


def save_config_to_yaml(config_instance, config_save_dir):
    """
    Saves the attributes of a configuration class instance to a YAML file.

    **Input**:
        - config_instance: configuration instance (TrainConfig or TestConfig class)
        - config_save_dir: path to the directory where the YAML file will be saved
    """
    config_dict = {
        key: value
        for key, value in config_instance.__class__.__dict__.items()
        if not key.startswith('_') and not callable(value)
    }

    json_string = json.dumps(config_dict, indent=4)
    yaml_data = yaml.safe_load(json_string)

    with open(config_save_dir, 'w') as f:
        yaml.safe_dump(yaml_data, f, sort_keys=False)


def main():
    
    config = TrainConfig()
    model = YOLO(config.model_name)

    yolo_type = config.model_name.split('.')[0]
    run_name = f"{yolo_type}_train"
    project_dir = config.runs_dir

    results = model.train(
        data=config.data_yaml,
        epochs=config.num_epochs,
        imgsz=config.imgsz,
        batch=config.batch_size,
        device=config.device,
        patience=config.patience,
        lr0=config.lr0,
        weight_decay=config.weight_decay,
        cos_lr=True, # try using a cosine annealing lr schedule
        val=True,
        plots=True,
        name=run_name,
        project=project_dir
    )

    # avoid AttributeError in case the used YOLO version does not provide this attribute
    if hasattr(results, 'save_dir') and results.save_dir:
        actual_run_dir = results.save_dir
    else:
        actual_run_dir = os.path.join(project_dir, run_name)

    config_output_path = os.path.join(actual_run_dir, "train_config.yaml")
    save_config_to_yaml(config_instance=config, config_save_dir=config_output_path)


if __name__ == '__main__':
    main()