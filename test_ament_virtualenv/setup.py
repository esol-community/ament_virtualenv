from setuptools import setup
import setuptools.command.install
import ament_virtualenv.install

package_name = 'test_ament_virtualenv'

class InstallCommand(setuptools.command.install.install):
    def run(self):
        super().run()
        ament_virtualenv.install.install_venv(self.install_base, package_name)
        # instead of self.install_base we may also use:
        # self.config_vars['platbase'] or self.config_vars['base']
        return

setup(
    cmdclass={
        'install': InstallCommand
    },
    name=package_name,
    version='0.0.2',
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_ament_virtualenv = test_ament_virtualenv.test_ament_virtualenv:main',
        ],
    },
)
