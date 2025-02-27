#!/usr/bin/env python3

# Copyright 2025 Open Source Robotics Foundation, Inc.
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

# Vendored from https://github.com/osrf/nexus

import atexit
from contextlib import contextmanager
import os
import signal
import subprocess
from subprocess import Popen


def kill_process(proc: Popen):
    if proc.poll() is None:
        os.kill(-proc.pid, signal.SIGINT)
        proc.wait()


@contextmanager
def managed_process(*args, **kwargs):
    """
    Create a context managed process group.

    A context managed process group that kills the process group when the
    context or when the script is exited. This avoid zombie processes in
    `ros2 run`, `ros2 launch` and other process launchers that do not kill
    their subprocesses on exit.

    :param args: The arguments to pass to `subprocess.Popen`.
    :param kwargs: The keyword arguments to pass to `subprocess.Popen`, except
        `start_new_session` which will always be `True`.
    """
    with subprocess.Popen(*args, **kwargs, start_new_session=True) as proc:
        def exit_cb():
            kill_process(proc)

        atexit.register(exit_cb)
        try:
            yield proc
        finally:
            exit_cb()
            atexit.unregister(exit_cb)
