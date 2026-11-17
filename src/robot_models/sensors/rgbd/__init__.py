"""The depth camera type: a stereo pair that reports a depth image registered to a colour stream, as one part.

Its ladder (`ladder.py`) climbs from ideal stereo triangulation through disparity noise, depth bias and a mount that
loosens with vibration, to a grey-box correction of the depth image.
"""
