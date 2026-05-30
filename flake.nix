{
  description = "Playground Nix Dev Environment";

  inputs = {
    nixpkgs.url = "nixpkgs/nixos-26.05";
  };

  outputs =
    { nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
      llvmPkg = pkgs.llvmPackages_22;
    in
    {
      devShells.${system} = rec {

        llvm = (pkgs.mkShellNoCC.override { stdenv = llvmPkg.stdenv; }) {

          packages = [
            pkgs.nixfmt
            pkgs.clang-tools

            llvmPkg.lld
            llvmPkg.lldb
            llvmPkg.clang

            pkgs.cmake
            pkgs.ninja

            pkgs.elfutils
            pkgs.shaderc
            pkgs.vulkan-tools
            pkgs.openssl
            pkgs.vulkan-loader
            pkgs.libxcb
            pkgs.libxcb-keysyms
            pkgs.libxkbcommon
            pkgs.alsa-lib
          ];

          # Configure runtime dependencies.
          NIX_LDFLAGS = "-rpath ${
            pkgs.lib.makeLibraryPath [
              pkgs.elfutils
              pkgs.openssl
              pkgs.shaderc
              pkgs.vulkan-loader
              pkgs.libxcb
              pkgs.libxcb-keysyms
              pkgs.libxkbcommon
              pkgs.alsa-lib
            ]
          }";

          VK_LOADER_DEBUG = "error"; # error,warn,info
        };

        default = llvm;
      };
    };
}
