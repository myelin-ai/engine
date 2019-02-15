#!/usr/bin/env bash

set -e

zip -0 "$coverage_zip_file_name" `find . \( -name 'myelin_engine*.gc*' \) -print`
grcov "$coverage_zip_file_name" \
       -s engine \
       -t lcov \
       --llvm \
       --branch \
       --ignore-not-existing \
       --ignore-dir '/*' \
       > lcov.info
