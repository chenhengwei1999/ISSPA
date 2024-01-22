from setuptools import setup
from glob import glob


setup(
    name='isspa',
    version='0.1.0',
    packages=['isspa'],
    install_requires=[
        # 依赖项列表
    ],
    scripts=glob('scripts/isspa/*.py'),
    package_dir={'': 'src'},
)