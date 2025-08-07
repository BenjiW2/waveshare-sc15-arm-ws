from setuptools import setup

setup(
    name='stservo_sdk_py',
    version='0.1.0',
    packages=['STservo_sdk'],
    
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/stservo_sdk_py']),
        ('share/stservo_sdk_py', [
            'package.xml',
            'plugin_description.xml',
        ]),
    ],

    scripts=[
        'scripts/servo_config.py',
        'scripts/sc15_id_burner.py'
    ],

    install_requires=[
	'setuptools',
        'pyserial>=3.5',
        'pyudev>=0.24',

    ],

    zip_safe=True,
    maintainer='Benji',
    maintainer_email='benjiw@stanford.edu',
    description='SDK and tools for fuckass waveshare servos on ROS 2',
    license='MIT',
    tests_require=['pytest']
,
    entry_points={
        'console_scripts': [
            # 'servo_config = servo_config:main',
        ],
	'ros2_control.plugin': [
      	    'sc15_hw_iface = stservo_sdk_py.hardware.sc15_hw_iface:SC15Bus',
        ],
    },
)
