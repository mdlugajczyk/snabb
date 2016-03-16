with import <nixpkgs> {};

stdenv.mkDerivation rec {
  name = "snabb-manual";
  src = ./../../.;

  buildInputs = [ ditaa pandoc git
   (texlive.combine {
      inherit (texlive) scheme-small luatex luatexbase sectsty titlesec cprotect bigfoot titling droid;
    })
  ];

  buildPhase = ''
    # needed for font cache
    export TEXMFCACHE=`pwd`

    make book -C src
  '';

  installPhase = ''
    mkdir -p $out/share/doc
    cp src/doc/snabbswitch.* $out/share/doc
  '';
}
