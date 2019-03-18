//! This crate contains the physical engine of
//! the simulation, as well as the objects that reside
//! within it.

#![feature(specialization)]
#![feature(non_exhaustive)]
#![feature(box_syntax)]
#![deny(
    rust_2018_idioms,
    missing_debug_implementations,
    missing_docs,
    clippy::doc_markdown,
    clippy::unimplemented
)]
#![cfg_attr(test, allow(clippy::float_cmp))]

pub use myelin_geometry as geometry;

pub mod object;
mod object_builder;
pub mod prelude;
mod private;
pub mod simulation;
pub mod world_interactor;
