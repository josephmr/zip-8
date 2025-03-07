{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/eaf382d9db0c636a7cdb454e36968662b0f151e2";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem
      (system:
        let
          pkgs = import nixpkgs { inherit system; };
        in
        {
          devShell = with pkgs; mkShell {
            buildInputs = [
              zig
              watchexec
            ];
          };
        }
      );
}
