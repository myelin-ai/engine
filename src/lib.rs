//! This crate contains the physical engine of
//! the simulation, as well as the objects that reside
//! within it.

#![feature(specialization)]
#![feature(non_exhaustive)]
#![feature(box_syntax)]
#![feature(trait_alias)]
#![warn(missing_docs, clippy::dbg_macro, clippy::unimplemented)]
#![deny(
    rust_2018_idioms,
    future_incompatible,
    missing_debug_implementations,
    clippy::doc_markdown,
    clippy::default_trait_access,
    clippy::enum_glob_use,
    clippy::needless_borrow,
    clippy::large_digit_groups,
    clippy::explicit_into_iter_loop
)]
#![cfg_attr(test, allow(clippy::float_cmp))]

pub use myelin_geometry as geometry;

pub mod object;
mod object_builder;
pub mod prelude;
mod private;
pub mod simulation;
pub mod world_interactor;
