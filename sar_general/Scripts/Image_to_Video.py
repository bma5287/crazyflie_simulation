import os
import subprocess

def images_to_video(image_directory, output_video, frame_rate=30):
    # Check if the image directory exists
    if not os.path.exists(image_directory):
        raise ValueError(f"Directory {image_directory} does not exist")

    # Construct the ffmpeg command
    # This assumes the image files are named in the format `image_%d.png` where `%d` is an integer.
    command = [
        'ffmpeg', 
        '-framerate', str(frame_rate), 
        '-i', os.path.join(image_directory, 'image_%d.png'), 
        '-c:v', 'libx264', 
        '-pix_fmt', 'yuv420p', 
        output_video
    ]

    # Run the command
    subprocess.run(command, check=True)

# Example usage:
image_directory = "/home/bhabas/Run_1/Gazebo_Recording_1"
output_video = image_directory + '/output_video.mp4'
frame_rate = 10  # Adjust frame rate as needed

images_to_video(image_directory, output_video, frame_rate)