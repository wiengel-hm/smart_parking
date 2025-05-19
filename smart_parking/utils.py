import subprocess

def replay_bagfile(bagfile_path):
    try:
        # Define remapping and filtering options correctly
        remap_option = '/rc/ackermann_cmd:=/autonomous/ackermann_cmd'  # Use --remap for ROS 2 remapping
        filter_option = '/rc/ackermann_cmd'  # Example filter option

        # Start `ros2 bag play` with correct remapping and filtering syntax
        process = subprocess.Popen([
            'ros2', 'bag', 'play', bagfile_path,
            '--remap', remap_option,
            '--topics', filter_option
        ])

        # Wait for the process to complete
        process.communicate()

    except Exception as e:
        print(f"An error occurred: {e}")

def get_mapping():

    index2frame = {
        0: 'USS_SRB',  # Side Right Back
        1: 'USS_SRF',  # Side Right Front
        2: 'USS_FR"',  # Front Right
        3: 'USS_FC',   # Front Center
        4: 'USS_FL',   # Front Left
        5: 'USS_SLF',  # Side Left Front
        6: 'USS_SLB',  # Side Left Back
        7: 'USS_BL',   # Back Left
        8: 'USS_BC',   # Back Center
        9: 'USS_BR'    # Back Right
    }

    # Create reverse mapping from frame name to index
    frame2index = {frame: i for i, frame in index2frame.items()}

    return frame2index