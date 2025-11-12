from setuptools import find_packages, setup

package_name = 'team_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben',
    maintainer_email='benorr3@gmail.com',
    description='RBE-500 Team Project source code package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fwd_kinematics = team_project.fwd_kinematics:main',
            'inv_kinematics = team_project.inv_kinematics:main',
        ],
    },
)
