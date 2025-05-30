#! /bin/python
import sys
import subprocess
import json
import os
import re

original_script = "/opt/ros/humble/lib/rosidl_typesupport_fastrtps_c/rosidl_typesupport_fastrtps_c"
args = sys.argv[1:]

json_file = [arg for arg in args if arg.endswith('.json')][0]

proc = subprocess.run(['python3', original_script] + args)
proc.check_returncode()

def replace_namespace_and_type(content: str):
    # Replace namespace type
    content = content.replace('"px4_msgs_old"', '"px4_msgs"')
    content = content.replace('"px4_msgs_old::msg"', '"px4_msgs::msg"')
    # Replace versioned type with non-versioned one
    content = re.sub(r'("[a-zA-Z0-9]+)V[0-9]+"', '\\1"', content)
    # Services
    content = content.replace('"px4_msgs_old::srv"', '"px4_msgs::srv"')
    content = re.sub(r'("[a-zA-Z0-9]+)V[0-9]+_Request"', '\\1_Request"', content)
    content = re.sub(r'("[a-zA-Z0-9]+)V[0-9]+_Response"', '\\1_Response"', content)
    return content

with open(json_file, 'r') as f:
    data = json.load(f)
    output_dir = data['output_dir']

    # Iterate files recursively
    for root, dirs, files in os.walk(output_dir):
        for file in files:
            with open(os.path.join(root, file), 'r+') as f:
                content = f.read()
                f.seek(0)
                f.write(replace_namespace_and_type(content))
                f.truncate()
