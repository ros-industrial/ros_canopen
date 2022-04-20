from setuptools import setup

package_name = 'canopen_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christoph',
    maintainer_email='christoph.hellmann.santos@ipa.fraunhofer.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cyclic_tester = canopen_utils.cyclic_tester:main'
        ],
    },
)
