#!/usr/bin/env python3
import os
import subprocess
import argparse 
from concurrent.futures import ThreadPoolExecutor
import tempfile

def get_files_from_directories(base_path='/media/art-pi/'):
    """Return a list of files from all directories in the base path."""
    files = []
    for root, _, filenames in os.walk(base_path):  
        for filename in filenames:
            files.append(os.path.join(root, filename))
    return files


def compress_file(file, file_destination):
    cmd = [ 'zip', '-j', file_destination, file]
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f'Successfully compressed {file}')
    else:
        print(f'Error compressing {file_destination}: {result.stderr}')
        
def compress_files(base_path, compressed_path):
    '''Comopress all files in rosbags diretory and put them into crosbags directory'''
    files = get_files_from_directories(base_path)
    print('Compressing files...')
    num_files = len(files)
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as executor:
        for index, file in enumerate(files):
            file_destination = file.replace(base_path, compressed_path)
            file_destination = file_destination + '.zip'
            
            if not os.path.exists(file_destination):
                os.makedirs(os.path.dirname(file_destination), exist_ok=True)
                
            executor.submit(compress_file, file, file_destination)
            
def decompress_file(file, file_destination):
    cmd = ['unzip', '-j', '-o', file, '-d', os.path.dirname(file_destination)]    
    print(f'Decompressing {file}')
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f'Successfully decompressed {file}')
    else:
        print(f'Error decompressing {file}: {result.stderr}')
            
def decompress_files(compressed_path, decompressed_path):
    '''Decompress all files in compressed directory and put them into decompressed directory'''
    files = get_files_from_directories(compressed_path)
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as executor:
        for file in files:
            file_destination = file.replace(compressed_path, decompressed_path)
            file_destination = file_destination.replace('.zip', '')
            
            if not os.path.exists(file_destination):
                os.makedirs(os.path.dirname(file_destination), exist_ok=True)
                
            executor.submit(decompress_file, file, file_destination)
            

def rclone_list_files(remote_path: str, local_path: str):
    '''List all files in the remote directory'''
    print("Fetching metadata, please wait...")
    cmd = ['rclone', 'lsf', remote_path, '--recursive', '--files-only']
    result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    local_files = get_files_from_directories(local_path)
    files = result.stdout.splitlines()
    local_files = [file.replace(local_path, '')+'.zip' for file in local_files]
    files = [file for file in files if file not in local_files]
    
    return files
  
def rclone_download_file(remote_file_path: str, local_file_path: str):
    '''Download a single file from remote directory and put it into local directory'''
    print(f'Downloading {remote_file_path}...')
    cmd = ['rclone', 'copy', remote_file_path, local_file_path]
    result = subprocess.run(cmd, capture_output=True, text=True, check=True)
    if result.returncode == 0:
        print(f'Successfully downloaded {remote_file_path}')
    else:
        print(f'Error downloading {remote_file_path}: {result.stderr}')
        
def process_file(remote_file_path: str, local_file_path: str):
    '''Download a single file from remote directory and put it into local directory'''
    with tempfile.TemporaryDirectory() as temp_dir:
        rclone_download_file(remote_file_path, temp_dir)
        decompress_files(temp_dir, os.path.dirname(local_file_path))
        os.remove(temp_dir)

def procces_files(remote_path: str, local_path: str, max_downloads: int):
    '''Download all files from remote directory and put them into local directory'''
    files = rclone_list_files(remote_path, local_path)
    
    with ThreadPoolExecutor(max_workers=max_downloads) as executor:
        for file in files:
            executor.submit(process_file, f"{remote_path}{file}", f"{local_path}{file}")
        
            

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('--base_path', '-i', help='Path to the directory containing rosbags')
    args.add_argument('--secondary_path', '-o', help='Path to the directory where compressed files will be stored')
    args.add_argument('--extract', '-e', action='store_true', help='Decompress files')
    args.add_argument('--download', '-d', action='store_true', help='Download files from remote directory')
    args.add_argument('--max-downloads', '-m', type=int, action='store', help='Maximum number of files to download', default=5)
    args = args.parse_args()
    
    base_path = args.base_path
    secondary_path = args.secondary_path
    
    if base_path is None and secondary_path is None:
        print('Please provide a base path and a compressed path')
        exit(1)

    if args.extract:
        decompress_files(base_path, secondary_path)
    elif args.download:
        procces_files(base_path, secondary_path, args.max_downloads)
    else:
        compress_files(base_path, secondary_path)