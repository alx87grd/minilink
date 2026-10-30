"""Every public name has a test: the inventory finds no facade name that no test file uses."""

from benchmarks.public_names import inventory, untested


def test_every_public_name_is_used_by_a_test():
    table = inventory()
    missing = untested(table)

    assert not missing, (
        f"{len(missing)} public name(s) no test uses: {missing}. Each needs a test of "
        "its contract (python benchmarks/public_names.py --untested lists them)"
    )
