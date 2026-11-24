import os
from glob import glob

from setuptools import setup

package_name = 'rehab_pack'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Martins',
    maintainer_email='lucaspmartins14@gmail.com',
    description='Mastro de reabilitação no CR10 com célula FA7155.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'rehab_admittance = rehab_pack.admittance_node:main',
        'rehab_force_source = rehab_pack.force_source:main',
        'rehab_logger = rehab_pack.session_logger:main',
        'rehab_gui = rehab_pack.rehab_gui:main',
        'rehab_mirror = rehab_pack.real_mirror:main',
        'rehab_preflight = rehab_pack.preflight:main',
    ]},
)
