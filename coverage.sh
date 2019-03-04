#!/usr/bin/env bash

set -e

zip -0 engine.zip `find . \( -name 'myelin_engine*.gc*' \) -print`
grcov engine.zip \
       -t lcov \
       --llvm \
       --branch \
       --ignore-not-existing \
       --ignore-dir '/*' \
       > lcov.info
