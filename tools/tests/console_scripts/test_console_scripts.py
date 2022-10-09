import subprocess

import pytest


@pytest.mark.parametrize("script",
                         [
                             "plot-data-log",
                             "plot-multi-body",
                             "plot-att-est",
                             "plot-hard-iron-offset",
                             "plot-motor-dynamics",
                             "plot-motor-thrust",
                             "plot-motor-vibrations",
                             "gui-linear-sim",
                             "gui-6dof-sim",
                             "gui-filter-design",
                         ])
def test_call_help_and_expect_exit_code_zero(script):
    subprocess.check_call([script, "--help"])
