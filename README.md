# ament_virtualenv

[![License: GPL v2](https://img.shields.io/badge/License-GPL%20v2-blue.svg)](https://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html)


Python virtual environment wrapper package, based on [catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv/).

This package provides a mechanism to:

- export python library requirements in `requirements.txt` format via `package.xml`.
- bundle a virtualenv within a ament package, inheriting requirements from any dependencies.
- wrap python scripts and tests in a ament package with a virtualenv loader.

At build time, CMake macros provided by this package will create a virtualenv, and create
wrapper scripts for any Python scripts in the package. Both will be included in any associated bloom artifacts.

This library is GPL licensed due to the inclusion of dh_virtualenv.


## Exporting python requirements

The package containing python modules with external library dependencies should define a `requirements.txt`:

```python
GitPython>=2.1.5
psutil>=5.2.2
wrapt>=1.10.10
```

Add an export to `package.xml`:

```xml
<export>
  <pip_requirements>requirements.txt</pip_requirements>
</export>
```

Make sure to install the requirements file in `CMakeLists.txt`:

```cmake
install(FILES requirements.txt
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})
```

If an ament package exports dependencies in a `requirements.txt` file, any dependent ament package that bundles a virtualenv (see below) will inherit those dependencies.

## Bundling virtualenv

It's possible to bundle all of an ament package's python requirements, as well as those of its ament dependencies, into a virtualenv.

This operation does not do any dependency resolution - similar to how `pip` operates, the topmost dependency declaration
'wins' (https://github.com/pypa/pip/issues/988).

Add an build dependency on ament_virtualenv to `package.xml`, as well as on any library packages you may want. Traditionally

```xml
<build_depend>ament_virtualenv</build_depend>

<!-- In an ament/python world, this would normally be an exec_depend. However, if `some_python_library` exports 
requirements.txt, it needs to be pulled in at build time as well -->
<depend>some_python_library</depend>
```

In CMakeLists.txt:

```cmake
# Make sure to find-package `ament_cmake_virtualenv`
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_virtualenv REQUIRED)


# Generate the virtualenv:
ament_generate_virtualenv()

# Must be called after target definitons
ament_package()
```


### Additional CMake Options

The following options are supported by `ament_generate_virtualenv()`:

```cmake
ament_generate_virtualenv(
  # Select an alternative version of the python interpreter - it must be installed on the system. Minor version is optional.
  PYTHON_VERSION 3.7  # Default 2

  # Choose not to use underlying system packages. This excludes any python packages installed by apt or system-pip from the environment.
  USE_SYSTEM_PACKAGES FALSE  # Default TRUE

  # Disable including pip requirements from catkin dependencies of this package.
  ISOLATE_REQUIREMENTS TRUE  # Default FALSE

  # Provide extra arguments to the underlying pip invocation
  EXTRA_PIP_ARGS
    --no-binary=:all:
    -vvv
)
