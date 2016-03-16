{ nixpkgs ? (import <nixpkgs> {}) }:

{
  doc = import ./manual.nix;
  manual =
    { doc }:
    nixpkgs.stdenv.mkDerivation {
      name = "manual";
      builder = ''
        echo "doc manual $out/share/doc/snabbswitch.pdf" \
          >> $out/nix-support/hydra-build-products;
      '';
    };
}

