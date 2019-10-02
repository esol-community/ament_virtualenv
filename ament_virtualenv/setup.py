from setuptools import find_packages
from setuptools import setup

package_name = 'ament_virtualenv'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    package_data={'': [
        'src/catkin_virtualenv',
        'src/dh_virtualenv',
    ]},
    zip_safe=True,
    author='Max Krichenbauer',
    author_email='v-krichenbauer7715@esol.co.jp',
    maintainer='Max Krichenbauer',
    maintainer_email='v-krichenbauer7715@esol.co.jp',
    url='https://github.com/ament/ament_lint',
    download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='',
    long_description="""\
.""",
    license='Apache License, Version 2.0',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'build_venv = ament_virtualenv.build_venv:main',
            'combine_requirements = ament_virtualenv.combine_requirements:main',
            'glob_requirements = ament_virtualenv.glob_requirements:main',
        ],
    },
)
