{
  inputs = {
    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:nixos/nixpkgs/23.05";
  };

  outputs = { self, flake-utils, nixpkgs }:
    flake-utils.lib.eachDefaultSystem (system:
    let
      pkgs = nixpkgs.legacyPackages.${system};
      py = pkgs.python310Packages;
    in {
      devShell = pkgs.mkShell {
        nativeBuildInputs = [
          pkgs.clang-tools_16
          pkgs.platformio-core
          py.intelhex
          py.future
          py.prettytable
          py.jsonschema
        ];
      };
  });
}