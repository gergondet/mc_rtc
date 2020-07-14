#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import mc_rtc

if __name__ == "__main__":
    print("mc_rtc compile time version: {}".format(mc_rtc.MC_RTC_VERSION))
    print("mc_rtc run time version: {}".format(mc_rtc.version()))
