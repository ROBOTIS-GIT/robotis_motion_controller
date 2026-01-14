from setuptools import setup, find_packages

setup(
    name='motion_controller_core',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'numpy',
    ],
    extras_require={
        'test': ['pytest', 'matplotlib'],
    },
    author='Motion Controller Project',
    description='Core logic for Motion Controller Project (Pure Python)',
    license='Apache-2.0',
)
