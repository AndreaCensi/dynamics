import os
from setuptools import setup, find_packages

version = "0.1"

description = """""" 

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()
    
long_description = read('README.rst')
    

setup(name='PyDynamics',
      author="Andrea Censi",
      author_email="AUTHOR_EMAIL",
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
      download_url='http://github.com/GITHUB_USER/GITHUB_PROJECT/tarball/%s' % version,
      
      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=[ ],
      tests_require=['nose'],
      entry_points={},
)

