from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robotis_vuer'
authors_info = [
    ('Wonho Yun', 'ywh@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name=package_name,
    version='0.0.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='VR Publisher for Robotis',
    license='Apache License 2.0',
    extras_require={
    },
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_publisher_sg2 = robotis_vuer.vr_publisher_sg2:main',
            'vr_publisher_sh5 = robotis_vuer.vr_publisher_sh5:main',
        ],
    },
)
