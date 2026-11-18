"""Pytest plugin: record every matplotlib figure a test draws, as a data and an artist record."""

import hashlib
import json
import os

import numpy as np
import pytest

RECORDS = {}


def digest(values):
    """A short hash of an array's values, rounded so float noise below 1e-9 does not count."""
    try:
        a = np.ma.filled(np.ma.asarray(values, dtype=float), np.nan)
    except (TypeError, ValueError):
        return "obj:" + str(type(values).__name__)
    a = np.round(a, 9) + 0.0
    return hashlib.sha1(a.tobytes() + str(a.shape).encode()).hexdigest()[:12]


def axes_record(ax):
    """The data an axes shows, independent of which artist draws it, and its artists."""
    data = []
    kinds = []
    for line in ax.get_lines():
        data.append(digest(np.column_stack(line.get_data())))
        kinds.append("Line2D")
    for coll in ax.collections:
        kinds.append(type(coll).__name__)
        offsets = coll.get_offsets()
        if offsets is not None and len(offsets):
            data.append(digest(offsets))
        arr = coll.get_array()
        if arr is not None:
            data.append(digest(arr))
        if not len(offsets) and arr is None:
            for path in coll.get_paths()[:200]:
                data.append(digest(path.vertices))
    for image in ax.images:
        kinds.append("AxesImage")
        data.append(digest(image.get_array()))
    for patch in ax.patches:
        kinds.append(type(patch).__name__)
        verts = patch.get_patch_transform().transform(patch.get_path().vertices)
        data.append(digest(verts))
    texts = sorted(t.get_text() for t in ax.texts if t.get_text())
    legend = ax.get_legend()
    record = {
        "title": ax.get_title(),
        "xlabel": ax.get_xlabel(),
        "ylabel": ax.get_ylabel(),
        "xlim": [round(float(v), 6) for v in ax.get_xlim()],
        "ylim": [round(float(v), 6) for v in ax.get_ylim()],
        "xscale": ax.get_xscale(),
        "yscale": ax.get_yscale(),
        "texts": texts,
        "legend": sorted(t.get_text() for t in legend.get_texts()) if legend else [],
        "data": sorted(data),
    }
    if hasattr(ax, "get_zlim"):
        record["zlabel"] = ax.get_zlabel()
    return record, sorted(kinds)


def figure_record(fig):
    """One figure: its suptitle and its axes, colour bars set apart."""
    axes, artists = [], []
    for ax in fig.axes:
        if ax.get_label() == "<colorbar>":
            continue
        rec, kinds = axes_record(ax)
        axes.append(rec)
        artists.append(kinds)
    suptitle = fig._suptitle.get_text() if fig._suptitle else ""
    return {"suptitle": suptitle, "axes": axes}, artists


def capture(nodeid, figs):
    """Record figures not recorded yet for this test, in creation order."""
    entry = RECORDS.setdefault(nodeid, {"data": [], "artists": [], "seen": set()})
    for fig in figs:
        if id(fig) in entry["seen"]:
            continue
        entry["seen"].add(id(fig))
        data, artists = figure_record(fig)
        entry["data"].append(data)
        entry["artists"].append(artists)


CURRENT = {"nodeid": None, "before": set()}


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    original_close = plt.close

    def recording_close(fig=None):
        if CURRENT["nodeid"] is not None:
            if fig is None:
                figs = [plt.gcf()] if plt.get_fignums() else []
            elif fig == "all":
                figs = [plt.figure(n) for n in plt.get_fignums()]
            elif isinstance(fig, (int, str)):
                figs = [plt.figure(fig)] if fig in plt.get_fignums() else []
            else:
                figs = [fig]
            # Only the figures this test made: an older one belongs to its own test
            figs = [f for f in figs if f.number not in CURRENT["before"]]
            capture(CURRENT["nodeid"], figs)
        return original_close(fig)

    plt.close = recording_close
    try:
        from minilink.graphical.common.environment import override_env

        override_env("script")
    except ImportError:
        pass


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_call(item):
    import matplotlib.pyplot as plt

    CURRENT["nodeid"] = item.nodeid
    before = set(plt.get_fignums())
    CURRENT["before"] = before
    yield
    figs = [plt.figure(n) for n in plt.get_fignums() if n not in before]
    capture(item.nodeid, figs)
    CURRENT["nodeid"] = None


def pytest_sessionfinish(session):
    out = os.environ.get("BASELINE_OUT")
    if not out:
        return
    worker = os.environ.get("PYTEST_XDIST_WORKER", "main")
    clean = {
        k: {"data": v["data"], "artists": v["artists"]}
        for k, v in RECORDS.items()
        if v["data"]
    }
    with open(f"{out}.{worker}.json", "w") as fh:
        json.dump(clean, fh, sort_keys=True, indent=0)
