from setuptools import setup

package_name = 'cartpole_mujoco_panel'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manu',
    maintainer_email='manu@example.com',
    description='Steuerpanel fuer das MuJoCo Cart Pole',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_panel = cartpole_mujoco_panel.control_panel:main',
        ],
    },
)
