from setuptools import setup

package_name = 'limo_path_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suwon Lee',
    maintainer_email='suwon@kookmin.ac.kr',
    description='VFG path follower for LIMO robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_follower_node = limo_path_follower.path_follower_node:main',
            'orchestrator_node = limo_path_follower.orchestrator_node:main',
        ],
    },
)
