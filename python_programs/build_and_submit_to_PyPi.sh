#! /usr/bin/env bash

# let's copy the source files from the servomotor module into the area where we will build the package
cp -r servomotor PyPi_distribution/src/

# Let's install the build package, but first check if it is already installed and if it is then don't install it
if ! python3 -m pip list | grep -q "build"; then
    python3 -m pip install --upgrade build
fi

# let's build the package
cd PyPi_distribution
rm -rf dist
python3 -m build

# we need to make sure that twine is installed. If it is not installed then install it
if ! python3 -m pip list | grep -q "twine"; then
    python3 -m pip install --upgrade twine
fi

# let's upload the package to PyPi
python3 -m twine upload --repository testpypi dist/*

