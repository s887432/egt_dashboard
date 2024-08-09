#!/bin/bash
gfxconvert -s
gfxconvert -m
gfxconvert -i img navigation_left.png 
gfxconvert -i img navigation_right.png 
gfxconvert dashboard.svg 
gfxconvert -e

