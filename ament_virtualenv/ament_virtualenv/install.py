#!/usr/bin/env python
#
# Copyright 2019 eSol Co.,Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#
# \file      install
# \authors   Max Krichenbauer <v-krichenbauer7715@esol.co.jp>
# \copyright Copyright (c) (2019), eSol Co.,Ltd., All rights reserved.
#

import os
import stat
import sys
import subprocess
import shutil
import importlib
import unittest
import psutil


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


def install_venv(install_base, package_name, python_version='2'):
    venv_install_dir = os.path.join(install_base, 'venv')
    bin_dir = os.path.join(install_base, 'bin')
	# 
    # Build the virtual environment
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
    cmd = [
        python,
        build_venv,
        '--root-dir', venv_install_dir,
        '--requirements', generated_requirements,
        '--retries', '3',
        '--python-version', python_version,
        # '--use-system-packages',
        '--extra-pip-args', '\"-qq\"',
    ]
    ret = subprocess.check_output(cmd)    
    # 
    # Wrapper shell executables we installed
    for bin_file in os.listdir(bin_dir):
        if bin_file[-5:] == '-venv':
            continue # possible left-over from last installation
        # rename file from 'xxx' to 'xxx-venv'
        bin_path = os.path.join(bin_dir, bin_file)
        if not os.path.isfile(bin_path):
            continue
        os.rename(bin_path, bin_path+'-venv')
        # create new file with the name of the previous file
        with open(bin_path, "w") as f:
            f.write("#!/usr/bin/python3\n")
            f.write("import os\n")
            f.write("import sys\n")
            f.write("import subprocess\n")
            f.write("if __name__ == '__main__':\n")
            f.write("    dir_path = os.path.dirname(os.path.realpath(__file__))\n")
            f.write("    bin_path = os.path.join(dir_path, '" + bin_file + "-venv')\n")
            f.write("    cmd = '" + venv_install_dir + "/bin/python ' + bin_path\n")
            f.write("    sys.exit(subprocess.call(cmd, shell=True))\n")
        # change file permissions to executable
        st = os.stat(bin_path)
        os.chmod(bin_path, st.st_mode | stat.S_IEXEC | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def main(argv=sys.argv[1:]):
	return 0 # todo


if __name__ == "__main__":
    main()
