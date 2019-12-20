export ROS_DISTRO=
export CASK_DEPENDENCIES="gfortran"
export BREW_DEPENDENCIES="coreutils pkg-config gnu-sed wget python cmake doxygen libtool tinyxml2 geos boost eigen nanomsg yaml-cpp qt qwt"
export PIP_DEPENDENCIES="Cython coverage nose numpy"
export WITH_ROS_SUPPORT="false"

mc_rtc_extra_steps()
{
  if [ ! -f /etc/apt/sources.list.d/pierre-gergondet_ppa-multi-contact-unstable-trusty.list ]
  then
    sudo add-apt-repository ppa:pierre-gergondet+ppa/multi-contact-unstable
    sudo apt-get update
    sudo apt-get upgrade cmake libeigen3-dev -qq
  fi
  if [ ! -d $SOURCE_DIR/yaml-cpp ]
  then
    git clone --recursive https://github.com/jbeder/yaml-cpp $SOURCE_DIR/yaml-cpp
    mkdir -p $SOURCE_DIR/yaml-cpp/build
    cd $SOURCE_DIR/yaml-cpp/build
    cmake ../ -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DYAML_BUILD_SHARED_LIBS:BOOL=ON
    make -j${BUILD_CORE}
    sudo make install
  fi
  if [ ! -d $SOURCE_DIR/nanomsg ]
  then
    git clone --recursive https://github.com/nanomsg/nanomsg $SOURCE_DIR/nanomsg
    mkdir -p $SOURCE_DIR/nanomsg/build
    cd $SOURCE_DIR/nanomsg/build
    cmake ../ -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    make -j${BUILD_CORE}
    sudo make install
  fi
}
