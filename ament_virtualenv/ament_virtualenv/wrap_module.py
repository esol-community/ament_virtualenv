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
import argparse

def wrap_module(bin_path, venv_install_dir):
    if not os.path.isfile(bin_path):
        return -1
    bin_dir  = os.path.dirname(bin_path)
    bin_file = os.path.basename(bin_path)
    # rename file from 'xxx' to 'xxx-venv'
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
        f.write("    sys.exit(subprocess.call(cmd, shell=True))\n")
    # change file permissions to executable
    st = os.stat(bin_path)
    os.chmod(bin_path, st.st_mode | stat.S_IEXEC | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    return 0


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser()
    parser.add_argument('--module-path', required=True)
    parser.add_argument('--venv-install-dir', required=True)
    args, unknown = parser.parse_known_args()
    return wrap_module(
        bin_path=args.module_path,
        venv_install_dir=args.venv_install_dir
    )


if __name__ == "__main__":
    sys.exit(main())
