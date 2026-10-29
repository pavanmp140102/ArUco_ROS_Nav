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

"""Keep each end-to-end test's Gazebo private and make sure it is gone afterwards."""

import atexit
import os
import signal
import uuid


def kill_partition(token):
    """
    Kill every process whose environment carries the given Gazebo partition.

    The simulator server can outlive the launch that started it. A leftover server publishes a
    second /clock and fails the next docking scenario, so each test removes its own after the
    launch has shut down. A test killed by a signal never runs this, so a timeout can still
    leave a server behind; its partition keeps it away from the next test.
    """
    needle = ('GZ_PARTITION=%s' % token).encode()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open('/proc/%s/environ' % pid, 'rb') as f:
                if needle in f.read().split(b'\0'):
                    os.kill(int(pid), signal.SIGKILL)
        except OSError:
            continue  # the process ended meanwhile, or belongs to someone else


def isolate_gazebo():
    """
    Give this test its own Gazebo transport partition and clean it up when the test exits.

    A leftover server from an earlier scenario then cannot be seen by the next one, which keeps
    the docking runs independent of each other.
    """
    token = 'dockbot_%d_%s' % (os.getpid(), uuid.uuid4().hex[:8])
    os.environ['GZ_PARTITION'] = token
    # The test's own `gz service` calls must reach the server the launch starts, which is
    # pinned to loopback; the launch's SetEnvironmentVariable does not reach this process.
    os.environ['GZ_IP'] = '127.0.0.1'
    atexit.register(kill_partition, token)
