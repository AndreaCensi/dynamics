import os
from setuptools import setup, find_packages

version = "0.5"

description = """""" 

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()
    
long_description = read('README.rst')
    

setup(name='PyVehiclesDynamics',
      author="Andrea Censi",
      author_email="andrea@cds.caltech.edu",
      url='',
      
      description=description,
      long_description=long_description,
      keywords="PROJECT_KEYWORDS",
      license="PROJECT_LICENSE",
      
      classifiers=[
        'Development Status :: 4 - Beta',
        # 'Intended Audience :: Developers',
        # 'License :: OSI Approved :: GNU Library or Lesser General Public License (LGPL)',
        # 'Topic :: Software Development :: Quality Assurance',
        # 'Topic :: Software Development :: Documentation',
        # 'Topic :: Software Development :: Testing'
      ],

	  version=version,
      download_url='http://github.com/AndreaCensi/vehicles_dynamics/tarball/%s' % version,
      
      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=['PyGeometry', 'PyContracts'],
      tests_require=['nose'],
      entry_points={},
)

