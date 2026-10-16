"""RULES §5 as a ratchet: the textbook-rule backlog only shrinks."""

from tests.unittest.textbook_style import CHECKS, census, read_allowlist

REWRITE = "python tests/unittest/textbook_style.py --write"


def test_no_new_textbook_offences():
    found, allowed = census(), read_allowlist()
    new = sorted(key for key in found if found[key] > allowed.get(key, 0))
    lines = [f"{key} ({CHECKS[key.split(chr(9))[0]]})" for key in new]
    assert not new, "New textbook-rule offences:\n" + "\n".join(lines)


def test_allowlist_shrinks_with_the_code():
    found, allowed = census(), read_allowlist()
    stale = sorted(key for key in allowed if found.get(key, 0) < allowed[key])
    assert not stale, (
        f"These entries no longer offend; shrink the allowlist with `{REWRITE}`:\n"
        + "\n".join(stale)
    )
