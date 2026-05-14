"""Inject joint dynamics into UR5 URDF for Gazebo stiffness (WSL2 no ros2_control)"""
import re, sys
path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/ur5.urdf'
with open(path) as f:
    u = f.read()
u = re.sub(r'(<limit[^>]*/>)', r'\1\n      <dynamics damping="100" friction="50"/>', u)
with open(path, 'w') as f:
    f.write(u)
print('Injected joint dynamics into', path)
