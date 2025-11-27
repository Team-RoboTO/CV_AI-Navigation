import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from test import TestConfig
import shutil

def extract_metrics_data(root_dir):
    """
    Finds 'metrics.txt' files within the specified root_dir and extracts metrics 
    using simple string matching. Uses the containing directory name as the Test Run ID.
    
    **Input**:
        root_dir: path to the directory containing the run subdirectories
    """
    data = []
    METRIC_KEYS = ['mAP50', 'FPS', 'Avg confidence', 'Latency']
    
    # recursively search for all 'metrics.txt' files starting from root_dir
    for fpath in Path(root_dir).rglob('metrics.txt'):
        metrics = {'TestRunID': fpath.parent.name}
        found_metrics = 0
        
        try:
            with open(fpath, 'r') as f:
                content = f.read()

            for line in content.splitlines():
                if ':' in line:
                    key, value = line.split(':', 1)
                    # remove white spaces
                    key = key.strip()
                    value = value.strip()
                    
                    if key in METRIC_KEYS:
                        # extract number, handling units like "ms/image"
                        number_value = value.split(' ')[0]
                        metrics[key] = float(number_value)
                        found_metrics += 1
            
            if found_metrics >= 4:
                 data.append(metrics)
            
        except Exception as e:
            print(f"Warning: Could not process {fpath}: {e}")
            continue

    return pd.DataFrame(data)

def save_bar_plot(df, metric_col, y_label, m_dir):
    """
    Generates a Matplotlib bar plot for a single metric.
    
    **Input**:
        - df: DataFrame containg the metrics data
        - metric_col: metric from which we want to generate the bar plot
        - y_label: title for the bar plot image
        - m_dir: path to the metrics directory where the image will be saved
    
    """
    
    # ensure data is sorted from highest value to lowest
    df_sorted = df.sort_values(by=metric_col, ascending=False)
    
    fig, ax = plt.subplots(figsize=(10, 6))

    x_labels = df_sorted['TestRunID']
    y_values = df_sorted[metric_col]
    
    # plot bars
    bars = ax.bar(x_labels, y_values, color='skyblue')

    ax.set_title(f'Comparison of {y_label} by Test Run')
    ax.set_xlabel('Test Directory Run ID')
    ax.set_ylabel(y_label)
    plt.xticks(rotation=45, ha='right')

    # add value labels on top of bars
    for bar in bars:
        yval = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2, yval * 1.02, 
                f'{yval:.4g}', ha='center', va='bottom')
        
    img_name = f"{metric_col.replace(' ', '_')}_comparison.png"
    img_path = os.path.join(m_dir, img_name)

    plt.tight_layout()
    plt.savefig(img_path)
    plt.close(fig)


def main():
    
    config = TestConfig()
    root_dir=config.test_runs_dir
    metrics_dir = '../metrics'

    if os.path.exists(metrics_dir):
        shutil.rmtree(metrics_dir)
    
    os.makedirs(metrics_dir)

    df = extract_metrics_data(root_dir=root_dir)

    if df.empty:
        print(f"Error: No valid metrics data found in the directory or subdirectories of '{root_dir}'.")
        return

    print(f"Found {len(df)} run configurations. Generating comparison charts...")
    
    save_bar_plot(df, 'FPS', 'Frames Per Second (FPS)',  m_dir=metrics_dir)
    save_bar_plot(df, 'mAP50', 'Mean Average Precision at 50% IOU (mAP50)',  m_dir=metrics_dir)
    save_bar_plot(df, 'Latency', 'Latency (ms/image)',  m_dir=metrics_dir)
    save_bar_plot(df, 'Avg confidence', 'Average Confidence',  m_dir=metrics_dir)

    print(f"Charts saved as PNG files in {metrics_dir}")


if __name__ == '__main__':
    main() 
    