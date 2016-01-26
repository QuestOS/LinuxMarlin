#!/bin/sh

kill -9 `ps aux | grep marlin | grep -v grep | awk '{print $2}'`

