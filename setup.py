from setuptools import setup, find_packages

setup(
    name='swisssmartfarming',
    version='0.0.1',
    author='Sebastiano Rusca',
    author_email='sebastiano.rusca@gmail.com',
    url='https://github.com/binello7/swisssmartfarming',
    description=('Package containing scripts and tools for the Swiss Smart '
        'Farming Project.'),
    packages=find_packages(),
    python_requires='>=3.6',
)
