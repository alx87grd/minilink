"""Sphinx configuration for Minilink documentation."""

from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".mpl"))

project = "minilink"
author = "Minilink contributors"
copyright = "Minilink contributors"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]

autosummary_generate = True
autodoc_member_order = "bysource"
autodoc_typehints = "description"
napoleon_google_docstring = False
napoleon_numpy_docstring = True

# Docstring types such as ``u : array of shape (m,)`` turn the dimension name
# into a cross-reference, and ``m`` matches both ``System.m`` and
# ``Trajectory.m``. The build runs with ``-W``, so silence only that ambiguity;
# a duplicate object description still fails it.
suppress_warnings = ["ref.python"]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "plans", "reviews"]

# GitHub Pages project site (repo name = minilink).
html_baseurl = "https://alx87grd.github.io/minilink/"

html_theme = "furo"
html_title = "Minilink"
html_static_path = ["_static"]
html_css_files = ["custom.css"]

html_theme_options = {
    "source_repository": "https://github.com/alx87grd/minilink/",
    "source_branch": "main",
    "source_directory": "docs/",
    "sidebar_hide_name": False,
    "top_of_page_buttons": ["view", "edit"],
    "light_css_variables": {
        "color-brand-primary": "#2563eb",
        "color-brand-content": "#2563eb",
        "color-api-background": "#f8fafc",
        "color-api-pre-background": "#f1f5f9",
    },
    "dark_css_variables": {
        "color-brand-primary": "#60a5fa",
        "color-brand-content": "#93c5fd",
    },
}

html_show_sourcelink = True
