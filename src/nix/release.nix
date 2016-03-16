{ nixpkgs ? (import <nixpkgs> {}) }:

{
  doc = import ./manual.nix;
}

