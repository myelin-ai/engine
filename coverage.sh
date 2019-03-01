#!/usr/bin/env bash

set -e

zip -0 engine.zip `find . \( -name 'myelin_engine*.gc*' \) -print`
grcov engine.zip \
       -s engine \
       -t lcov \
       --llvm \
       --branch \
       --ignore-not-existing \
       --ignore-dir '/*' \
       > lcov_engine.info

zip -0 geometry.zip `find . \( -name 'myelin_geometry*.gc*' \) -print`
grcov geometry.zip \
       -s geometry \
       -t lcov \
       --llvm \
       --branch \
       --ignore-not-existing \
       --ignore-dir '/*' \
       > lcov_geometry.info
