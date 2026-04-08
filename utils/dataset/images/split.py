import os
import shutil
import random

def split_dataset(source_dir, dest_dir, split_ratio=0.2):
    # Ensure the destination directory exists
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)

    # Define common image extensions
    valid_extensions = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')
    
    # Get list of all image files in the source folder
    files = [f for f in os.listdir(source_dir) 
             if f.lower().endswith(valid_extensions)]
    
    # Randomize the file list
    random.shuffle(files)
    
    # Calculate how many files to move
    num_to_move = int(len(files) * split_ratio)
    files_to_move = files[:num_to_move]

    print(f"Found {len(files)} images. Moving {num_to_move} to '{dest_dir}'...")

    # Move the files
    for filename in files_to_move:
        source_path = os.path.join(source_dir, filename)
        dest_path = os.path.join(dest_dir, filename)
        shutil.move(source_path, dest_path)

    print("Transfer complete.")

# Run the function
split_dataset("temp-goal-test", "val")