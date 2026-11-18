"""The rigid chain (the bam_fb five-bar): its models, and its metrics from the same building blocks.

Its runs carry two joints (j2, c1) and the tool, the L2 tip as a pose; the tool error is the distance between the
target and measured tip positions.
"""

from __future__ import annotations

from robot_models.ladder import ladders

from integrated_servo import COMPARE
from measures import RowError

models = ladders()["rigid_body_chain"]

j2_error = RowError("j2 error", "deg", short="j2", rows="all", joint=0, description="joint j2's angle error on every row",
                    references=(COMPARE, "bam_fivebar.speed_ladder step comparisons"))
c1_error = RowError("c1 error", "deg", short="c1", rows="all", joint=1, description="joint c1's angle error on every row",
                    references=(COMPARE, "bam_fivebar.speed_ladder step comparisons"))
tool_error = RowError("tool error", "mm", short="tool", rows="all", scale=1e3, streams=("tool_target", "tool_measured"),
                      description="the distance between the target and measured L2 tip on every row",
                      references=(COMPARE, "evo APE (Grupp 2017)"))

metrics = [j2_error.rms(), c1_error.rms(), tool_error.rms(), tool_error.max()]
