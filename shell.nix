{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = [
    pkgs.python313

    pkgs.python313Packages.matplotlib

    pkgs.python313Packages.jax
    pkgs.python313Packages.optax
    pkgs.python313Packages.flax

    pkgs.wayland-scanner
    pkgs.wayland
    pkgs.glfw3

    pkgs.SDL2
    pkgs.perf
  ];
}
# { pkgs ? import <nixpkgs> { } }:
#
# let
#   dlopenLibraries = with pkgs; [
#     libxkbcommon
#
#     # GPU backend
#     vulkan-loader
#     # libGL
#
#     # Window system
#     wayland
#
#     SDL2
#     # xorg.libX11
#     # xorg.libXcursor
#     # xorg.libXi
#   ];
# in pkgs.mkShell {
#   nativeBuildInputs = with pkgs; [
#     #cargo
#     #rustc
#   ];
#
#   env.RUSTFLAGS = "-C link-arg=-Wl,-rpath,${pkgs.lib.makeLibraryPath dlopenLibraries}";
# }
