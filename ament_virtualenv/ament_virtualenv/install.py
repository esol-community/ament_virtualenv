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
import argparse

try:
    from ament_virtualenv.glob_requirements import glob_requirements
    from ament_virtualenv.combine_requirements import combine_requirements
    from ament_virtualenv.build_venv import build_venv
    ament_virtualenv_import_failed = False
except:
    ament_virtualenv_import_failed = True
#

def find_program(name='build_venv.py', package='ament_virtualenv'):
    '''
    Helper function to find the modules that are part of this
    package (glob_requirements, combine_requirements, build_venv),
    in cases where a direct import failes due to python path issues.
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
        return 1

    # glob_requirements --package-name ament_cmake_haros
    if ament_virtualenv_import_failed == True:
        # Fallback: try to find the command line script and run it
        glob_requirements_py = find_program(name='glob_requirements.py', package='ament_virtualenv')
        if not glob_requirements_py:
            print("ERROR: Failed to locate glob_requirements", file=sys.stderr)
            return 1
        cmd = [
            python, 
            glob_requirements_py,
            '--package-name',
            package_name
        ]
        requirements_list = subprocess.check_output(cmd)
        requirements_list = requirements_list.decode("utf-8").strip()
    else:
        # Use the module directly
        requirements_list = glob_requirements(package_name=package_name, no_deps=False)
    # ^ glob_requirements
    #
    # combine_requirements --requirements-list a/requirements.txt;b/requirements.txt
    #                      --output-file x/generated_requirements.txt
    generated_requirements = '/tmp/test_ament_virtualenv-generated_requirements.txt'
    if ament_virtualenv_import_failed:
        combine_requirements_py = find_program(name='combine_requirements.py', package='ament_virtualenv')
        if not combine_requirements_py:
            print("ERROR: Failed to locate combine_requirements", file=sys.stderr)
            return 1
        cmd = [
            python,
            combine_requirements_py,
            '--requirements-list',
            requirements_list,
            '--output-file',
            generated_requirements
        ]
        subprocess.check_output(cmd)
    else:
        requirements_files = []
        for requirements_file in requirements_list.split(';'):
            requirements_files.append(open(requirements_file, 'r'))
        generated_requirements_file = open(generated_requirements, 'w')
        combine_requirements(
            requirements_list=requirements_files,
            output_file=generated_requirements_file
        )
        for requirements_file in requirements_files:
            requirements_file.close()
        generated_requirements_file.close()
    # ^ combine_requirements
    #
    if ament_virtualenv_import_failed:
        build_venv_py = find_program(name='build_venv.py', package='ament_virtualenv')
        if not build_venv_py:
            print("ERROR: Failed to locate build_venv", file=sys.stderr)
            return 1
        cmd = [
            python,
            build_venv_py,
            '--root-dir', venv_install_dir,
            '--requirements', generated_requirements,
            '--retries', '3',
            '--python-version', python_version,
            # '--use-system-packages',
            '--extra-pip-args', '\"-qq\"',
        ]
        ret = subprocess.check_output(cmd)
    else:
        ret = build_venv(
            root_dir=venv_install_dir,
            python_version=python_version,
            requirements_filename=generated_requirements,
            use_system_packages=False,
            extra_pip_args="-qq",
            retries=3
        )
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
        venv_rel_path = os.path.relpath(venv_install_dir, bin_dir)
        # create new file with the name of the previous file
        with open(bin_path, "w") as f:
            f.write("#!/usr/bin/python3\n")
            f.write("import os\n")
            f.write("import sys\n")
            f.write("import subprocess\n")
            f.write("if __name__ == '__main__':\n")
            f.write("    dir_path = os.path.dirname(os.path.realpath(__file__))\n")
            f.write("    bin_path = os.path.join(dir_path, '" + bin_file + "-venv')\n")
            f.write("    vpy_path = os.path.abspath(os.path.join(dir_path, '" + venv_rel_path +"'))\n")
            f.write("    vpy_path = os.path.join(vpy_path, 'bin', 'python')\n")
            f.write("    cmd = vpy_path + ' ' + bin_path\n")
            f.write("    if len(sys.argv) > 1:\n")
            f.write("        cmd += ' ' + ' '.join(sys.argv[1:])\n")
            f.write("    sys.exit(subprocess.call(cmd, shell=True))\n")
        # change file permissions to executable
        st = os.stat(bin_path)
        os.chmod(bin_path, st.st_mode | stat.S_IEXEC | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    #
    return 0

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser()
    parser.add_argument('--install-base', required=True)
    parser.add_argument('--package-name', required=True)
    parser.add_argument('--python-version', required=True)
    args, unknown = parser.parse_known_args()
    return install_venv(
        install_base=args.install_base,
        package_name=args.package_name,
        python_version=args.python_version
    )


if __name__ == "__main__":
    sys.exit(main())
