#!/usr/bin/env python3

import os
import ssl
from setuptools import setup

# Ignore ssl if it fails
if (not os.environ.get('PYTHONHTTPSVERIFY', '') and
        getattr(ssl, '_create_unverified_context', None)):
    ssl._create_default_https_context = ssl._create_unverified_context

setup(name='dvl_service',
      version='0.1.0',
      description='Waterlinked A-50 DVL service',
      license='MIT',
      install_requires=['python3-nmap == 1.5.4', 'loguru == 0.5.3', 'Flask == 1.0.3','MarkupSafe == 0.23','itsdangerous == 0.24','Jinja2 == 2.10', 'click == 7.1.2', 'Werkzeug==1.0.1', 'requests'])
