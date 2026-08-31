from setuptools import setup
import os
from glob import glob

package_name = 'touch_pack'

setup(
    name=package_name,
    version='0.3.0',
    packages=['touch_pack'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*.stl') + glob('meshes/*.STL')),
        # A calibração da célula axial VIAJA COM O PACOTE. Ela mora em
        # `<repo>/sensors/`, fora deste diretório, porque é um registro de
        # medição versionado e não um recurso de código — mas sem instalá-la
        # um deploy que leve só o `install/` perde a reta, e sem reta o
        # force_receiver não publica força nenhuma. Instalar é o que faz "a
        # mesma calibração em qualquer computador" valer sem ninguém copiar
        # arquivo à mão. Ver constants._resolve_lc_calib_file.
        (os.path.join('share', package_name, 'sensors'),
            glob('../../sensors/load_cell_calib.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lucas Martins',
    maintainer_email='lucaspmartins14@gmail.com',
    description=('Plataforma de palpação tátil — CR10 + COVVI Index FT, '
                 'reproduzindo o protocolo de Gupta et al. 2021.'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'palpation_gui     = touch_pack.palpation_gui:main',
            'tactile_explorer  = touch_pack.tactile_explorer:main',
            'palpation_logger  = touch_pack.palpation_logger:main',
            'palpation_report  = touch_pack.palpation_report:main',
            'real_pose_sync    = touch_pack.real_pose_sync:main',
            'force_receiver    = touch_pack.force_receiver_node:main',
            'ft_receiver       = touch_pack.ft_receiver_node:main',
            'sim_force_bridge  = touch_pack.sim_force_bridge:main',
            'kinematic_attacher = touch_pack.kinematic_attacher:main',
            'touch_receiver    = touch_pack.touch_receiver_node:main',
            'force_sync        = touch_pack.force_sync_node:main',
            'mirror_node       = touch_pack.mirror_node:main',
            'latency_probe     = touch_pack.latency_probe:main',
            'lc_health_probe   = touch_pack.lc_health_probe:main',
        ],
    },
)
