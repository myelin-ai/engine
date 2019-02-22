//! This crate contains the physical engine of
//! the simulation, as well as the objects that reside
//! within it.

#![feature(specialization, non_exhaustive, box_syntax)]
#![deny(
    rust_2018_idioms,
    missing_debug_implementations,
    missing_docs,
    clippy::doc_markdown,
    clippy::unimplemented
)]
#![cfg_attr(test, allow(clippy::float_cmp))]

pub mod object;
mod object_builder;
pub mod prelude;
pub mod simulation;
pub mod world_interactor;
