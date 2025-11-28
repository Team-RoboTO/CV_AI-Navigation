import os
import yaml
import shutil
from roboflow import Roboflow

def download_dataset(dataset_path="../dataset", data_yaml_path="../dataset/data.yaml"):
    """
    Downloads the dataset from Roboflow based on the configuration in data.yaml file.

    **Input**:
        - dataset_path:
        - data_yaml_path: 
    """
    # load data.yaml to get Roboflow credentials
    try:
        with open(data_yaml_path, 'r') as f:
            data_config = yaml.safe_load(f)
        
        roboflow_config = data_config.get('roboflow')
        if not roboflow_config:
            print("Error: 'roboflow' section not found in data.yaml.")
            return

        workspace_name = roboflow_config['workspace']
        project_name = roboflow_config['project']
        version = roboflow_config['version']
        download_format = 'coco'
        
    except FileNotFoundError:
        print(f"Error: {data_yaml_path} not found.")
        return
    
    # get API Key from Environment Variable
    api_key = os.environ.get("ROBOFLOW_API_KEY")
    if not api_key:
        print("Error: ROBOFLOW_API_KEY environment variable not set.")
        print("Please obtain your key from Roboflow and set it.")
        return

    # download the Project
    print(f"Connecting to Roboflow project: {project_name} (v{version})")
    
    rf = Roboflow(api_key=api_key)
    project = rf.workspace(workspace_name).project(project_name)
    
    temp_download_dir = f"./{project_name}-{version}"
    _ = project.version(version).download(download_format, location=temp_download_dir)
    print(f"Dataset download path: {temp_download_dir}")

    # ensure the target directory (../dataset) exists
    os.makedirs(dataset_path, exist_ok=True)

    # remove old dataset splits if present
    for folder_name in ['train', 'valid', 'test']:
        old_dir = os.path.join(dataset_path, folder_name)
        if os.path.exists(old_dir):
            shutil.rmtree(old_dir)
    
    # move the subfolders (train, valid, test) and any downloaded file (skip data.yaml, which is assumed to already exist)
    for item_name in os.listdir(temp_download_dir):
        src = os.path.join(temp_download_dir, item_name)
        dst = os.path.join(dataset_path, item_name)
        
        if item_name == 'data.yaml':
            continue 
        
        shutil.move(src, dst)
    
    print(f"\nSuccessfully downloaded dataset into {dataset_path}")

    # cleanup the temporary download folder
    shutil.rmtree(temp_download_dir)


def main():
    download_dataset(dataset_path="../dataset", data_yaml_path="../dataset/data.yaml")


if __name__ == "__main__":
    main()