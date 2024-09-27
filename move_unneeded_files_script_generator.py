#!/usr/bin/env python3

# this program will use git status to get a list of all untracked files, which are the files that follow the following text: 
# Untracked files:
#   (use "git add <file>..." to include in what will be committed)
# From all those untracked files, we will ignore the ones that start with unneeded_files
# all other files will be moved to the unneeded_files folder
# this program should first gather all the files that will be moved and print them to the screen
# it should then let the user review the list and the user should then either press CTRL+C to cancel the operation or press ENTER to proceed


import os
import subprocess
import shutil

# Function to get untracked files
def get_untracked_files():
    try:
        result = subprocess.run(['git', 'status'], capture_output=True, text=True, check=True)
        output = result.stdout
        untracked_files = []
        lines = output.splitlines()
        recording = False
        
        for line in lines:
            if line.startswith('Untracked files:'):
                recording = True
                continue
            if recording:
                if line.strip() == '':
                    break
                # Extract only valid file paths
                file_path = line.split()[-1]
                if file_path and not file_path.startswith('unneeded_files') and file_path != 'committed)':
                    untracked_files.append(file_path)
        
        return untracked_files
    except subprocess.CalledProcessError as e:
        print("Error while executing git status:", e)
        return []

# Function to move files while preserving the directory structure
def move_file_with_structure(src, base_dest):
    # Determine destination path
    relative_path = os.path.relpath(src, start=os.getcwd())
    dest = os.path.join(base_dest, relative_path)

    # Create destination directory if it doesn't exist
    os.makedirs(os.path.dirname(dest), exist_ok=True)

    shutil.move(src, dest)

# Main function
def main():
    untracked_files = get_untracked_files()
    unneeded_folder = 'unneeded_files'

    # Print files to be moved
    print("The following untracked files will be moved to the 'unneeded_files' folder:")
    files_to_move = [f for f in untracked_files]
    
    for file in files_to_move:
        print(file)

    # User review
    input("Press ENTER to proceed or CTRL+C to cancel...")

    # Move files while preserving the structure
    for file in files_to_move:
        if os.path.exists(file):
            move_file_with_structure(file, unneeded_folder)
        else:
            print(f"File not found: {file}")

if __name__ == "__main__":
    main()
