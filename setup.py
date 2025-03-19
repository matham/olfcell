from setuptools import setup, find_packages
import olfcell

with open('README.rst') as fh:
    long_description = fh.read()

setup(
    name='olfcell',
    version=olfcell.__version__,
    author='Matthew Einhorn',
    author_email='matt@einhorn.dev',
    url='https://matham.github.io/olfcell/',
    license='MIT',
    description='Olfaction plus cell labeling.',
    long_description=long_description,
    classifiers=['License :: OSI Approved :: MIT License',
                 'Topic :: Scientific/Engineering',
                 'Topic :: System :: Hardware',
                 'Programming Language :: Python :: 3.10',
                 ],
    packages=find_packages(),
    install_requires=[
        'base_kivy_app', 'kivy', 'pyserial',
        'trio', 'pymoa-remote', 'pymoa',
        'tree-config', 'kivy_trio', 'smbus2', 'gpiozero'],
    extras_require={
        'dev': [
            'pytest>=3.6', 'pytest-cov', 'flake8', 'sphinx-rtd-theme',
            'coveralls', 'pytest-trio',
            'pytest-kivy', 'pytest-dependency', 'docutils',
            'sphinx'],
    },
    package_data={'olfcell': ['data/*', '*.kv']},
    entry_points={'console_scripts': ['olfcell=olfcell.main:run_app']},
)
