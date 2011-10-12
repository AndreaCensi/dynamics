import os
from setuptools import setup, find_packages

version = "1.0"

description = """""" 

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()
    
long_description = read('README.rst')
    

setup(name='PyVehiclesDynamics',
      author="Andrea Censi",
      author_email="andrea@cds.caltech.edu",
      url='http://github.com/AndreaCensi/dynamics/',
      
      description=description,
      long_description=long_description,
      keywords="", # TODO: fill
      license="",
      
      classifiers=[
        'Development Status :: 4 - Beta',
        # 'Intended Audience :: Developers',
        # 'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
        # 'Topic :: Software Development :: Quality Assurance',
        # 'Topic :: Software Development :: Documentation',
        # 'Topic :: Software Development :: Testing'
      ],

	  version=version,
      download_url='http://github.com/AndreaCensi/dynamics/tarball/%s' % version,
      
      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=[
        'PyGeometry>=1.0,<2', 
        'PyContracts>=1.2,<2'
      ],
      tests_require=['nose'],
      entry_points={},
)

