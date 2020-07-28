from conans import ConanFile, CMake, tools
import os
import subprocess

class MCRTCTestConan(ConanFile):
    requires = "mc_rtc/latest@multi-contact/dev"
    settings = "os", "arch", "compiler", "build_type"
    generators = "cmake"

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def test(self):
        if not tools.cross_building(self.settings):
            os.chdir("bin")
            self.run(".%sexample" % os.sep)
            subprocess.check_call(['python', os.path.join(os.path.dirname(__file__), 'test.py')])
