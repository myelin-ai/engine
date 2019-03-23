# Myelin Engine

[![Build Status](https://travis-ci.com/myelin-ai/engine.svg?branch=master)](https://travis-ci.com/myelin-ai/engine)
[![Latest Version](https://img.shields.io/crates/v/myelin-engine.svg)](https://crates.io/crates/myelin-engine)
[![Documentation](https://docs.rs/myelin-engine/badge.svg)](https://docs.rs/myelin-engine)
[![dependency status](https://deps.rs/repo/github/myelin-ai/engine/status.svg)](https://deps.rs/repo/github/myelin-ai/engine)
[![Code Coverage](https://codecov.io/gh/myelin-ai/engine/branch/master/graph/badge.svg)](https://codecov.io/gh/myelin-ai/engine)

A general purpose 2D simulation engine for applications such as physics simulations or video games.

## Features

- Efficent 2D physics simulation via [nphysics](https://github.com/rustsim/nphysics/)
- A nice layer of abstraction, allowing you to concentrate on high-level concepts instead of gritty details
- Stable functionality, guaranteed by a vast test suite
- The possibility to inject own implementations of core internal components, allowing full unit testability of your application
