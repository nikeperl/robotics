from setuptools import find_packages, setup

package_name = 'fullname_msgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/FullNameMessage.msg']),
        ('share/' + package_name + '/srv', ['srv/FullNameSumService.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikelau',
    maintainer_email='n.zelenkov@g.nsu.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
