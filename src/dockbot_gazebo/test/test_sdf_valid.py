# Copyright 2026 Pavan Patil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check that every world and model parses with `gz sdf -k` (test S3 for the simulation files)."""

import glob
import os
import subprocess

import pytest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
MODELS = os.path.join(ROOT, 'models')
FILES = sorted(glob.glob(os.path.join(ROOT, 'worlds', '*.sdf')) +
               glob.glob(os.path.join(MODELS, '*', 'model.sdf')))


@pytest.mark.parametrize('path', FILES, ids=lambda p: os.path.relpath(p, ROOT))
def test_sdf_is_valid(path):
    """A world or model that does not parse would stop Gazebo before any docking starts."""
    env = dict(os.environ, SDF_PATH=MODELS, GZ_SIM_RESOURCE_PATH=MODELS)
    result = subprocess.run(['gz', 'sdf', '-k', path], capture_output=True, text=True, env=env)
    assert result.returncode == 0 and 'Valid' in result.stdout, result.stdout + result.stderr
