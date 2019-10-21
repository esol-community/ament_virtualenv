from setuptools import setup
import distutils.command.build
# alternative: import distutils.command.build_py
# alternative: import setuptools.command.build_py
import os
import sys
import subprocess
import shutil
import importlib
import unittest

package_name = 'test_ament_virtualenv'


class TestVirtualenv(unittest.TestCase):
    def run(self):
        requests = importlib.import_module("requests")
        self.assertEquals(requests.__version__, "2.20.1")
    #
#

def find_program(name='build_venv.py', package='ament_virtualenv'):
    '''
    '''
    ament_prefix_path = os.environ.get("AMENT_PREFIX_PATH")
    if not ament_prefix_path:
        return None
    for path in ament_prefix_path.split(os.pathsep):
        if not path.endswith(os.path.sep + package):
            continue
        for root, subdirs, files in os.walk(path, topdown=True):
            for file in files:
                if file == name:
                    return os.path.abspath(os.path.join(path, root, file))
    # else: not found
    return None
#


class BuildCommand(distutils.command.build.build):
    def run(self):
        python = shutil.which("python")
        if not python:
            print("ERROR: Failed to locate python", file=sys.stderr)
            return

        # glob_requirements --package-name ament_cmake_haros
        glob_requirements = find_program(name='glob_requirements.py', package='ament_virtualenv')
        if not glob_requirements:
            print("ERROR: Failed to locate glob_requirements", file=sys.stderr)
            return
        cmd = [
            python, 
            glob_requirements,
            '--package-name',
            package_name
        ]
        requirements_list = subprocess.check_output(cmd)
        requirements_list = requirements_list.decode("utf-8").strip()

        # combine_requirements --requirements-list a/requirements.txt;b/requirements.txt --output-file x/generated_requirements.txt
        combine_requirements = find_program(name='combine_requirements.py', package='ament_virtualenv')
        if not combine_requirements:
            print("ERROR: Failed to locate combine_requirements", file=sys.stderr)
            return
        generated_requirements = '/tmp/test_ament_virtualenv-generated_requirements.txt'
        cmd = [
            python,
            combine_requirements,
            '--requirements-list',
            requirements_list,
            '--output-file',
            generated_requirements
        ]
        subprocess.check_output(cmd)

        build_venv = find_program(name='build_venv.py', package='ament_virtualenv')
        if not build_venv:
            print("ERROR: Failed to locate build_venv", file=sys.stderr)
            return
        venv_install_dir = '/tmp/test_ament_virtualenv-venv'
        cmd = [
            python,
            build_venv,
            '--root-dir', venv_install_dir,
            '--requirements', generated_requirements,
            '--retries', '3',
            '--python-version', '2',
            # '--use-system-packages',
            '--extra-pip-args', '\"-qq\"',
        ]
        subprocess.check_output(cmd)
        # Check if we're actually in the virtualenv
        # and requirements have been met
        test = TestVirtualenv()
        test.run()
        # Now perform the normal setup operation
        super().run()
    # ^ def run()
# ^ class BuildPyCommand()



setup(
    cmdclass={
        'build': BuildCommand,
    },
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/'+package_name, ['package.xml', 'requirements.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    author='Max Krichenbauer',
    author_email='v-krichenbauer7715@esol.co.jp',
    maintainer='Max Krichenbauer',
    maintainer_email='v-krichenbauer7715@esol.co.jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Example of using ament_virtualenv.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'test_ament_virtualenv = test_ament_virtualenv.test_ament_virtualenv:main',
        ],
    },
)
