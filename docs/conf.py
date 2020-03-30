# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))

from datetime import datetime
import sphinx.ext.autodoc
import sensirion_sensorbridge_i2c_sfm

# -- Project information -----------------------------------------------------

project = u'Sensirion SFM I²C Python Driver for use with the Sensorbridge'
copyright = u'{} Sensirion AG, Switzerland'.format(datetime.now().year)
author = u'Sensirion AG'


# The short X.Y version
version = sensirion_sensorbridge_i2c_sfm.__version__
# The full version, including alpha/beta/rc tags
release = sensirion_sensorbridge_i2c_sfm.__version__


# -- General configuration ---------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# needs_sphinx = '1.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.inheritance_diagram',
    'sphinx.ext.githubpages',
    'sphinx.ext.intersphinx',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = None

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = [u'_build', 'Thumbs.db', '.DS_Store', 'generated']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
#
# html_theme_options = {}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``.
#
# html_sidebars = {}

html_favicon = 'favicon.ico'


# -- Extension configuration -------------------------------------------------

autodoc_member_order = 'bysource'

autodoc_default_flags = [
    'members',
    'special-members',      # To see __init__()
    'inherited-members',    # To see the methods from base classes
]


def autodoc_skip_member(app, what, name, obj, skip, options):
    whitelist = ('__init__', '__call__',)
    exclude = name.startswith('__') and (name not in whitelist)
    # blacklist members
    blacklist = ()
    return skip or exclude or name in blacklist


def setup(app):
    app.connect('autodoc-skip-member', autodoc_skip_member)


# Workaround for "=None" documentation of instance attributes
# (see https://github.com/sphinx-doc/sphinx/issues/2044)
sphinx.ext.autodoc.InstanceAttributeDocumenter.add_directive_header = \
    sphinx.ext.autodoc.ClassLevelDocumenter.add_directive_header

scv_whitelist_branches = ('master',)

scv_grm_exclude = ('.gitignore', '.nojekyll')

intersphinx_mapping = {
}
