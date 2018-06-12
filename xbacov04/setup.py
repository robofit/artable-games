#!/usr/bin/env python
# -*- coding: utf-8 -*-

##################################################################################
##
## @file       setup.py
## @date       11. 03. 2018
## @brief      Application setup file inspired by art_projected_gui package
## @author     Juraj Bačovčin (xbacov04@stud.fit.vutbr.cz)
##
##################################################################################

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

## Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['xbacov04', 'xbacov04.helpers'],
    package_dir={
        'xbacov04': 'src/xbacov04',
        'xbacov04.helpers': 'src/xbacov04/helpers'
        })

setup(**setup_args)

##################################################################################
## End of file setup.py
##################################################################################
