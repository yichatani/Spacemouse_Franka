import os 
import re
import zarr 
import numpy as np

## Script to convert data collected in npy to zarr for model training with batch processing

data_path = '/home/rmx/franka_ws/panda-py/data'  # path of collected demonstrations
save_path = '/home/rmx/franka_ws/panda-py/data.zarr'  # path to save zarr data

# Open or create the Zarr root group
zarr_root = zarr.open_group(save_path, mode='a')

# Check for 'data' group
if 'data' not in zarr_root:
    zarr_data = zarr_root.create_group('data')
    print("Created 'data' group")
else:
    zarr_data = zarr_root['data']
    print("'data' group already exists")

# Check for 'meta' group
if 'meta' not in zarr_root:
    zarr_meta = zarr_root.create_group('meta')
    print("Created 'meta' group")
else:
    zarr_meta = zarr_root['meta']
    print("'meta' group already exists")

compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)

# Regular expression to match and extract episode number
episode_pattern = re.compile(r'episode_(\d+)')

# List all items in the main folder
all_items = os.listdir(data_path)
print("all_items:", all_items)

# Initialize arrays to store batches' data
rgb_image_array_batch = []
eef_pose_array_batch = []
action_array_batch = []
episode_end_batch = []

# Batch size
batch_size = 1000

# Find and sort episode folders
episode_folders = sorted(
    [item for item in all_items if os.path.isdir(os.path.join(data_path, item)) and episode_pattern.match(item)],
    key=lambda x: int(episode_pattern.match(x).group(1))
)
print("episode_folders:", episode_folders)
# Gather data from every episode in batches
for episode_folder in episode_folders:
    episode_path = os.path.join(data_path, episode_folder)
    print(f"Opening folder: {episode_path}")

    # Load data
    rgb_image = np.load(os.path.join(episode_path, "rgb.npy"))
    eef_pose = np.load(os.path.join(episode_path, "eef_pose.npy"))
    # action = np.load(os.path.join(episode_path, "joint_action.npy"))

    # Process each batch within the episode
    for i in range(0, len(rgb_image), batch_size):
        # Get the batch
        batch_rgb_image = rgb_image[i:i + batch_size]
        batch_eef_pose = eef_pose[i:i + batch_size]
        # batch_action = action[i:i + batch_size]

        # Append batch data to the batch arrays
        rgb_image_array_batch.append(batch_rgb_image)
        eef_pose_array_batch.append(batch_eef_pose)
        # action_array_batch.append(batch_action)

        # Track the episode end index
        if episode_end_batch:
            episode_end_batch.append(episode_end_batch[-1] + batch_rgb_image.shape[0])
        else:
            episode_end_batch.append(batch_rgb_image.shape[0])

    print(f"Processed episode: {episode_folder}")

# Convert lists to numpy arrays after processing all episodes
rgb_image_array_batch = np.concatenate(rgb_image_array_batch, axis=0)
eef_pose_array_batch = np.concatenate(eef_pose_array_batch, axis=0)
# action_array_batch = np.concatenate(action_array_batch, axis=0)
episode_end_batch = np.array(episode_end_batch, dtype=object)

# Define chunk sizes
rgb_image_chunk_size = (batch_size, rgb_image_array_batch.shape[1], rgb_image_array_batch.shape[2], rgb_image_array_batch.shape[3])
# action_chunk_size = (batch_size, action_array_batch.shape[1])
eef_pose_chunk_size = (batch_size, eef_pose_array_batch.shape[1])

# Create zarr datasets
zarr_data.create_dataset('img', data=rgb_image_array_batch, chunks=rgb_image_chunk_size, dtype='uint8', overwrite=True, compressor=compressor)
# zarr_data.create_dataset('action', data=action_array_batch, chunks=action_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
zarr_data.create_dataset('eef_pose', data=eef_pose_array_batch, chunks=eef_pose_chunk_size, dtype='float32', overwrite=True, compressor=compressor)

# Save episode end indices
zarr_meta.create_dataset('episode_ends', data=episode_end_batch, chunks=(batch_size,), dtype='int64', overwrite=True, compressor=compressor)

print("Data conversion to zarr completed successfully.")