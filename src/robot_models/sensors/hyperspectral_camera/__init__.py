"""A line-scan (pushbroom) spectral camera over a belt: one line of the belt at a time, each pixel split into spectral bands.

Its ladder (PEP 0004) climbs from the ideal bands and their dark and white references, through band widths, smile and
keystone, and the belt-swept line geometry, to references that drift with deployment hours.
"""
