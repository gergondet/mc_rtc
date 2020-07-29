# -*- coding: utf-8 -*-
#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from conans import python_requires, CMake
import conans.tools as tools
from conans.tools import SystemPackageTool, os_info
import os
import shutil

base = python_requires("Eigen3ToPython/latest@multi-contact/dev")

class MCRTCConan(base.Eigen3ToPythonConan):
    name = "mc_rtc"
    version = "1.4.0"
    description = "Real time control of robots using constrained optimization"
    topics = ("robotics", "control", "optimization", "python")
    url = "https://github.com/jrl-umi3218/mc_rtc"
    homepage = "https://jrl-umi3218.github.io/mc_rtc/"
    author = "Pierre Gergondet <pierre.gergondet@gmail.com>"
    license = "BSD-2-Clause"
    exports = ["LICENSE"]
    exports_sources = [".gitignore", "3rd-party/*", "CMakeLists.txt", "conan/CMakeLists.txt", "conan/FindBoost.cmake", "binding/*", "cmake/*", "CMakeModules/*", "doc/*", "etc/*", "include/*", "plugins/*", "src/*", "utils/*"]
    generators = ["cmake_paths"]
    settings = "os", "arch", "compiler", "build_type"
    no_copy_source = True

    requires = (
        "mc_rtc_data/latest@multi-contact/dev",
        "Tasks/latest@multi-contact/dev",
        "eigen-quadprog/latest@multi-contact/dev",
        "hpp-spline/latest@multi-contact/3rd-party",
        "geos/latest@multi-contact/3rd-party",
        "spdlog/latest@multi-contact/3rd-party",
        "nanomsg/latest@multi-contact/3rd-party"
    )

    def package_info(self):
        self.cpp_info.libs = tools.collect_libs(self)
        self.env_info.LD_LIBRARY_PATH.append(os.path.join(self.package_folder, 'lib'))
        self.env_info.PYTHONPATH.append(self._extra_python_path())

    def system_requirements(self):
        installer = SystemPackageTool()
        if os_info.linux_distro == "ubuntu" or os_info.linux_distro == "debian":
            #FIXME Install ROS here?
            installer.install("libltdl-dev")
        elif os_info.is_macos:
            installer.install("coreutils")

    def source(self):
        base.Eigen3ToPythonConan.source(self)
        shutil.move(os.path.join("conan", "FindBoost.cmake"), "CMakeModules/")
        # Make sure we find conan's Boost not system Boost
        pattern = 'include(CMakeFindDependencyMacro)'
        replacement = '''if(CONAN_BOOST_ROOT)
  set(BOOST_ROOT "${{CONAN_BOOST_ROOT}}")
else()
  set(BOOST_ROOT "${{PACKAGE_PREFIX_DIR}}")
endif()
set(Boost_NO_SYSTEM_PATHS ON)
list(APPEND CMAKE_MODULE_PATH "${{CMAKE_CURRENT_LIST_DIR}}")
{}'''.format(pattern)
        tools.replace_in_file('cmake/Config.cmake.in', pattern, replacement)
        # Use and install the up-to-date FindBoost.cmake
        pattern = 'add_subdirectory(src)'
        replacement = '''{}
install(FILES CMakeModules/FindBoost.cmake DESTINATION lib/cmake/mc_rtc)'''.format(pattern)
        tools.replace_in_file('CMakeListsOriginal.txt', pattern, replacement)

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def _configure_cmake(self):
        os.environ['PATH'] =  self._extra_path() + os.pathsep + os.environ.get('PATH', '')
        os.environ['PYTHONPATH'] =  self._extra_python_path() + os.pathsep + os.environ.get('PYTHONPATH', '')
        cmake = CMake(self)
        cmake.definitions['DISABLE_TESTS'] = True
        cmake.definitions['CMAKE_BUILD_TYPE'] = self.settings.get_safe("build_type", "Release")
        cmake.definitions['PIP_INSTALL_PREFIX'] = self.package_folder
        cmake.definitions['PYTHON_BINDING_BUILD_PYTHON2_AND_PYTHON3'] = base.enable_python2_and_python3(self.options)
        cmake.definitions['DISABLE_ROS'] = True
        cmake.definitions['INSTALL_DOCUMENTATION'] = False
        cmake.configure()
        return cmake
