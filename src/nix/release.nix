{ nixpkgs ? (import <nixpkgs> {}) }:

{
  doc = import ./manual.nix;
  manual =
    { doc }:
    nixpkgs.stdenv.mkDerivation {
      name = "manual";
      builder = ''
        mkdir -p $out/nix-support
        echo "doc-pdf manual $out/share/doc/snabbswitch.pdf" \
          >> $out/nix-support/hydra-build-products;
      '';
    };
}

