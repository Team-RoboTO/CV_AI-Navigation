## Folder description

This folder contains the code for training and testing yolo models.
In particular, three scripts are provided:

- `train.py`: trains a yolo model with the specified configuration (`TrainConfig`)
- `test.py`: performs inference on a trained yolo model with the specified configuration (`TestConfig`)
- `download_dataset.py`: export and saves a dataset from RoboFlow to the `dataset` folder (this folder should contain a valid `data.yaml` file before running the script, otherwise it won't work)

Inside `TestConfig` is possible to set brightness and zoom in/out max percentages to test the trained model under different lighting conditions and from different perspectives.

---

## How to run the code

### 1. Install dependencies

This code had been tested using `Python-3.10`, but should work for `Python-3.8+`. The required python packages can be installed via `pip`:

```bash
pip install ultralytics opencv-python numpy pyyaml roboflow
```

Then, inside `cv_models_scripts/scripts` run on a terminal:

```bash
python3 download_dataset.py
```

---

### 2. Download a dataset

Be sure to have exported your Roboflow API Key as an environment variable by running the following on a terminal:

```bash
export ROBOFLOW_API_KEY="YOUR_KEY_HERE"
```

If you are using the *fish* shell, do this instead:

```bash
set -gx ROBOFLOW_API_KEY "YOUR_KEY_HERE"
```

Then open a terminal, go inside `cv_models_scripts/scripts` via `cd` and run:

```bash
python3 download_dataset.py
```

---

### 3. Perform training

Update the `TrainConfig` instance as desired, then open a terminal, go inside `cv_models_scripts/scripts` via `cd` and run:

```bash
python3 train.py
```

---

### 4. Perform inference

Update the `TestConfig` instance as desired, then open a terminal, go inside `cv_models_scripts/scripts` via `cd` and run:

```bash
python3 test.py
```

---

## Notes on reproducibility

- **configuration**: both `train.py` and `test.py` automatically save their exact configuration settings (`train_config.yaml` and `test_config.yaml`) within their respective output folders to ensure full experiment reproducibility
- **artifacts**: the final evaluation metrics (`metrics.txt`) explicitly record the path to the original training configuration that generated the model used for testing