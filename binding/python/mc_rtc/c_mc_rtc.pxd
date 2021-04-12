#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from libcpp.utility cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map as cppmap  # Careful map is a python built-in
from libcpp cimport bool as cppbool

ctypedef cppmap[string, vector[string]] mapStrVecStr
ctypedef cppmap[string, vector[double]] mapStrVecDouble

cimport eigen.c_eigen as c_eigen
cimport sva.c_sva as c_sva

# map taken from libcpp/map.pxd
cdef extern from "<mc_rtc/map.h>" namespace "mc_rtc" nogil:
    cdef cppclass map[T, U, COMPARE=*, ALLOCATOR=*, MaxLoadFactor100=*]:
        ctypedef T key_type
        ctypedef U mapped_type
        ctypedef pair[const T, U] value_type
        ctypedef COMPARE key_compare
        ctypedef ALLOCATOR allocator_type
        cppclass iterator:
            pair[T, U]& operator*()
            iterator operator++()
            iterator operator--()
            bint operator==(iterator)
            bint operator!=(iterator)
        cppclass reverse_iterator:
            pair[T, U]& operator*()
            iterator operator++()
            iterator operator--()
            bint operator==(reverse_iterator)
            bint operator!=(reverse_iterator)
        cppclass const_iterator(iterator):
            pass
        cppclass const_reverse_iterator(reverse_iterator):
            pass
        map() except +
        map(map&) except +
        #map(key_compare&)
        U& operator[](T&)
        #map& operator=(map&)
        bint operator==(map&, map&)
        bint operator!=(map&, map&)
        bint operator<(map&, map&)
        bint operator>(map&, map&)
        bint operator<=(map&, map&)
        bint operator>=(map&, map&)
        U& at(const T&) except +
        const U& const_at "at"(const T&) except +
        iterator begin()
        const_iterator const_begin "begin" ()
        void clear()
        size_t count(const T&)
        bint empty()
        iterator end()
        const_iterator const_end "end" ()
        pair[iterator, iterator] equal_range(const T&)
        #pair[const_iterator, const_iterator] equal_range(key_type&)
        void erase(iterator)
        void erase(iterator, iterator)
        size_t erase(const T&)
        iterator find(const T&)
        const_iterator const_find "find" (const T&)
        pair[iterator, bint] insert(pair[T, U]) except + # XXX pair[T,U]&
        iterator insert(iterator, pair[T, U]) except + # XXX pair[T,U]&
        #void insert(input_iterator, input_iterator)
        #key_compare key_comp()
        iterator lower_bound(const T&)
        const_iterator const_lower_bound "lower_bound"(const T&)
        size_t max_size()
        reverse_iterator rbegin()
        const_reverse_iterator const_rbegin "rbegin"()
        reverse_iterator rend()
        const_reverse_iterator const_rend "rend"()
        size_t size()
        void swap(map&)
        iterator upper_bound(const T&)
        const_iterator const_upper_bound "upper_bound"(const T&)
        #value_compare value_comp()


cdef extern from "<mc_rtc/config.h>" namespace "mc_rtc":
  const char * MC_ENV_DESCRIPTION_PATH
  const char * INSTALL_PREFIX
  const char * MC_ROBOTS_INSTALL_PREFIX
  const char * MC_CONTROLLER_INSTALL_PREFIX
  const char * DATA_PATH
  const char * CONF_PATH

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr(T*)
    T* get()

cdef extern from "<mc_rtc/log/Logger.h>" namespace "mc_rtc":
  cdef cppclass Logger:
    # Simplified from C++
    void addLogEntry[T](const string&, T get_fn)
    void removeLogEntry(const string&)

cdef extern from "<mc_rtc/Configuration.h>" namespace "mc_rtc":
  cdef cppclass Configuration:
    Configuration()
    Configuration(const string&)
    Configuration(const Configuration&)

    void load(const Configuration&)
    void load(const string&)
    void loadData(const string&)
    void save(const string&, cppbool)
    string dump(cppbool)

    cppbool has(const string&)
    Configuration operator()(const string&) except +
    Configuration operator[](int) except +
    vector[string] keys()
    cppbool empty()
    int size()

    Configuration add(const string&) except +
    void add(const string &, const Configuration &) except +

    Configuration array(const string&, int) except +
    void push(const Configuration &) except +

    cppbool remove(const string&)

cdef extern from "mc_rtc_wrapper.hpp":
  cdef cppclass function[T]:
    pass
  function[c_eigen.Vector3d] make_v3d_log_callback[T,U](T,U)
  function[double] make_double_log_callback[T,U](T,U)
  function[vector[double]] make_doublev_log_callback[T,U](T,U)
  function[c_eigen.Quaterniond] make_quat_log_callback[T,U](T,U)
  function[c_sva.PTransformd] make_pt_log_callback[T,U](T,U)
  function[c_sva.ForceVecd] make_fv_log_callback[T,U](T,U)
  function[string] make_string_log_callback[T,U](T,U)

  T get_config_as[T](Configuration&) except +
  T get_config_as[T](Configuration&, const T&) except +
  Configuration get_as_config[T](const T&) except +
  Configuration ConfigurationFromData(const string&)

  void set_loader_debug_suffix(const string&)
