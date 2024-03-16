from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.7.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mario Bozza',
    author_email='m.bozza@outlook.com',
    maintainer='Mario Bozza',
    maintainer_email='your@youremail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='control Ernest',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo = PID_controller.demo:main'
        ],
    },
)