
from setuptools import setup, find_packages

setup(
    name='jh1',
    version='0.1.0',
    description='Project Joseph Hoane (CS402 S25)',
    author='team_sd2_jh1',
    package_dir={'': 'src'},
    # packages=find_packages(include=['jh1', 'jh1.*']),
    packages=find_packages(where='src'),
    # install_requires=[],
    python_requires='>=3.8',
)