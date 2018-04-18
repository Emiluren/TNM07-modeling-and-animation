#!/bin/sh
find . -path ./build -prune -o -name "*.cpp" -print -or -name "*.h" -print | xargs etags --append
