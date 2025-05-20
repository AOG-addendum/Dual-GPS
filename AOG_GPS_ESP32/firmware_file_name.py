#!/usr/bin/python3

Import("env")

env.Replace(PROGNAME="dual_gps_79_firmware_%s" % env.GetProjectOption("custom_prog_version"))