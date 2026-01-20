# My_diffbot Documentation

This directory holds the source and configuration files used to generate the
My_diffbot documentation.

## Installation

Install `pip` and `venv` if not already installed:

``` bash
sudo apt install python3-pip python3-venv
```

Create a virtual environment and install the dependencies:

``` bash
python3 -m venv .venv
source .venv/bin/activate
pip3 install -r requirements.txt
```

## Build the Docs

Build the docs locally with

``` bash
make html
```

and you'll find the built docs entry point in `build/html/index.html`.
