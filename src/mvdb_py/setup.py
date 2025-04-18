from setuptools import find_packages, setup

package_name = 'mvdb_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'open3d',
        'pyyaml',
        'scipy'
        ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='peteris.racinskis@edi.lv',
    description='TODO: Package description',
    license='Apache-2.0',
    license_files=("LICENSE"),
    entry_points={
        'console_scripts': [
            'test = mvdb_py.utils:main',
        ],
    },
)
