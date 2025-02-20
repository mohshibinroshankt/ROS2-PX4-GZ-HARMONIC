from setuptools import find_packages
from setuptools import setup

setup(
    name='translation_node',
    version='0.0.0',
    packages=find_packages(
        include=('translation_node', 'translation_node.*')),
)
