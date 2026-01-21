import os

file_path = '/workspace/data/table_scene_lms400.pcd'

if not os.path.exists(file_path):
    print(f"Error: {file_path} not found.")
    exit(1)

print(f"File size: {os.path.getsize(file_path)} bytes")

with open(file_path, 'rb') as f:
    # Read first 500 bytes to check header
    header = f.read(500)
    try:
        print("\n--- Header (First 500 bytes) ---")
        print(header.decode('utf-8', errors='ignore'))
    except Exception as e:
        print(f"Error decoding header: {e}")
