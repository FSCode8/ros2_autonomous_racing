from setuptools import find_packages
from setuptools import setup

setup(
    name='image_controller',
    version='0.0.0',
    packages=find_packages(
        include=('image_controller', 'image_controller.*')),
)
