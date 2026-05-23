#!/usr/bin/python
#
# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain
from conan.tools.env import VirtualBuildEnv


class demos(ConanFile):
    python_requires = "libhal-bootstrap/[4.6.1]"
    python_requires_extend = "libhal-bootstrap.demo"

    options = {"board": ["ANY", None], "platform": ["ANY"], "variant": ["ANY"], "micromod_board": ["ANY"]}

    def generate(self):
        virt = VirtualBuildEnv(self)
        virt.generate()
        tc = CMakeToolchain(self)
        tc.cache_variables["PICO_BOARD"] = "libhal_picosdk"
        tc.generate()
        cmake = CMakeDeps(self)
        cmake.generate()

    def requirements(self):
        # self.requires("libhal-util/5.8.1")
        self.requires("libhal-picosdk/0.0.1")
        self.tool_requires("picotool/2.2.0")
