#!/usr/bin/env python
#
# Copyright 2019 eSOL Co.,Ltd.
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
# \file      glob_requirements
# \authors   Max Krichenbauer <v-krichenbauer7715@esol.co.jp>
# \copyright Copyright (c) (2019), eSol, All rights reserved.
#
from __future__ import print_function

import argparse
import sys
import os

try:
    from ament_virtualenv.package import parse_package
except ImportError:
    try:
        from package import parse_package
    except ImportError:
        from .package import parse_package

try:
    from queue import Queue
except ImportError:
    from Queue import Queue


def find_in_workspaces(project, file, workspaces=[]):
    # Add default workspace search paths
    ament_paths = os.environ.get('AMENT_PREFIX_PATH')
    if ament_paths is not None:
        # AMENT_PREFIX_PATH points at install/<package>
        ament_paths = ament_paths.split(os.pathsep)
        for path in ament_paths:
            if ((os.path.sep + 'install' + os.path.sep) in path or
               (os.path.sep + 'install_isolated' + os.path.sep) in path):
                workspaces.append(os.path.join(path, '..'))
                workspaces.append(os.path.join(path, '..', '..', 'src'))
                break
    if len(workspaces) == 0:
        # if AMENT_PREFIX_PATH wasn't set, we can fall back on
        # CMAKE_PREFIX_PATH (should contain the same information)
        cmake_paths = os.environ.get('CMAKE_PREFIX_PATH')
        if cmake_paths is not None:
            # CMAKE_PREFIX_PATH points at install/<package> or install_isolated/<package>
            cmake_paths = cmake_paths.split(os.pathsep)
            for path in cmake_paths:
                if ((os.path.sep + 'install' + os.path.sep) in path or
                   (os.path.sep + 'install_isolated' + os.path.sep) in path):
                    workspaces.append(os.path.join(path, '..'))
                    workspaces.append(os.path.join(path, '..', '..', 'src'))
                    break
    if len(workspaces) == 0:
        # COLCON_PREFIX_PATH points to the `install/` directory,
        # which is fine when ament_python is used as build tool
        # (ament_python copies the files right away),
        # but ament_cmake does not copy the files until after the
        # build, which is too late. So for ament_cmake we also
        # need to add the neighboring `src/` folder to the seach
        # (eg.: `install/../src/`)
        colcon_paths = os.environ.get('COLCON_PREFIX_PATH')
        if colcon_paths is not None:
            colcon_paths = colcon_paths.split(os.pathsep)
            for path in colcon_paths:
                if (os.path.sep + 'install') in path or (os.path.sep + 'install_isolated') in path:
                    workspaces.append(path)
                    workspaces.append(os.path.join(path, '..', 'src'))
    if len(workspaces) == 0:
        # final fallback: use working directory (usually src/<package>)
        path = os.getcwd()
        if (os.path.sep + 'src') in path:
            workspaces.append(path)
    if len(workspaces) == 0:
        raise RuntimeError(
            "[ament_virtualenv] Failed to find any workspaces." +
            "\nAMENT_PREFIX_PATH=" + os.environ.get('AMENT_PREFIX_PATH', 'NOT SET') +
            "\nCMAKE_PREFIX_PATH=" + os.environ.get('CMAKE_PREFIX_PATH', 'NOT SET') +
            "\nCOLCON_PREFIX_PATH=" + os.environ.get('COLCON_PREFIX_PATH', 'NOT SET') +
            "\nCWD=" + os.getcwd()
        )
    # now search the workspaces
    for workspace in (workspaces or []):
        for d, dirs, files in os.walk(workspace, topdown=True, followlinks=True):
            if (('CATKIN_IGNORE' in files) or
               ('COLCON_IGNORE' in files) or
               ('AMENT_IGNORE' in files)):
                del dirs[:]
                continue
            dirname = os.path.basename(d)
            if dirname == project and file in files:
                return os.path.join(workspace, d, file)
    # none found:
    return None
#


AMENT_VIRTUALENV_TAGNAME = "pip_requirements"


def parse_exported_requirements(package):
    # type: (catkin_pkg.package.Package) -> List[str]
    requirements_list = []
    for export in package.exports:
        if export.tagname == AMENT_VIRTUALENV_TAGNAME:
            requirements_path = find_in_workspaces(
                project=package.name,
                file=export.content
            )
            if not requirements_path:
                print(
                    ("[ERROR] ament_virtualenv "
                     "Package {package} declares <{tagname}> {file}, "
                     "which cannot be found in the package").format(
                        package=package.name,
                        tagname=AMENT_VIRTUALENV_TAGNAME,
                        file=export.content
                    ),
                    file=sys.stderr
                )
            else:
                requirements_list.append(requirements_path)
    return requirements_list


def process_package(package_name, soft_fail=True):
    # type: (str) -> List[str], List[str]
    workspaces = []
    package_path = find_in_workspaces(
        project=package_name,
        file="package.xml",
        workspaces=workspaces
    )
    if not package_path:
        if not soft_fail:
            raise RuntimeError("Failed to find package.xml for package " +
                               package_name + ' in ' + ';'.join(workspaces))
        else:
            # This is not an ament dependency
            return [], []
    else:
        package = parse_package(package_path)
        dependencies = package.build_depends + package.test_depends
        return parse_exported_requirements(package), dependencies


def glob_requirements(package_name, no_deps):
    # type: (str) -> int
    package_queue = Queue()
    package_queue.put(package_name)
    processed_packages = set()
    requirements_list = []

    while not package_queue.empty():
        queued_package = package_queue.get()

        if queued_package not in processed_packages:
            processed_packages.add(queued_package)
            requirements, dependencies = process_package(
                package_name=queued_package, soft_fail=(queued_package != package_name))
            requirements_list = requirements_list + requirements

            if not no_deps:
                for dependency in dependencies:
                    package_queue.put(dependency.name)
    return ';'.join(requirements_list)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser()
    parser.add_argument('--package-name', type=str, required=True)
    parser.add_argument('--no-deps', action="store_true")
    args, unknown = parser.parse_known_args()
    print(glob_requirements(**vars(args)))
    return 0
#


if __name__ == "__main__":
    sys.exit(main())
