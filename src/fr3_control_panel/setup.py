from setuptools import setup, find_packages
setup(name='fr3_control_panel', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/fr3_control_panel']),
                  ('share/fr3_control_panel', ['package.xml', 'README.md']),
                  ('share/fr3_control_panel/config', ['config/panel.yaml']),
                  ('share/fr3_control_panel/scripts', ['scripts/start_panel.sh', 'scripts/start_panel.bat']),
                  ('share/fr3_control_panel/launch', ['launch/mock_panel.launch.py'])],
      install_requires=['setuptools'],
      entry_points={'console_scripts': ['panel = fr3_control_panel.app:main']})
