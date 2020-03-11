#!/bin/bash

virtualenv -p /usr/bin/python2 venv2
source venv2/bin/activate
pip install -r requirements_py2.txt
deactivate

python3 -m venv venv3
source venv3/bin/activate
pip install -r requirements_py3.txt
