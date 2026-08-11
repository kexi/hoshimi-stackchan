{
  description = "compass-stackchan dev environment (CoreS3 / StackChan-BSP)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # nixpkgs の platformio-core は esptool の実行時依存を同梱しないため、
        # 自分で PYTHONPATH に足す。tool-esptoolpy の postinstall が
        # `python -m pip install` を呼ぶので pip も要る。
        #
        # Why not python312Packages: PlatformIO がどの Python で動くかは nixpkgs 側の
        # 都合で変わる (この lock では 3.14)。バージョンを直書きすると site-packages の
        # パスが噛み合わず黙って import に失敗するので、platformio-core 自身が使う
        # インタプリタから引く。
        pioPython = pkgs.platformio-core.python;
        pioEsptoolDeps = with pioPython.pkgs; [
          intelhex
          reedsolo
          bitstring
          bitarray # bitstring の実行時依存。欠けると import 時に落ちる
          pip
        ];
        pioEsptoolPath =
          pkgs.lib.concatMapStringsSep ":"
            (p: "${p}/${pioPython.sitePackages}")
            pioEsptoolDeps;
      in
      {
        devShells.default = pkgs.mkShell {
          packages = (with pkgs; [
            actionlint
            clang-tools
            cmake
            curl
            findutils
            git
            gitleaks
            just
            lefthook
            ninja
            pinact
            platformio
            python3
            python3Packages.pyserial
            ripgrep
            uv
          ]) ++ pkgs.lib.optionals pkgs.stdenv.isLinux [ pkgs.avahi ];

          shellHook = ''
            # ツールチェーンをリポジトリ局所に閉じ込め、$HOME を汚さない (.gitignore 済み)
            export PLATFORMIO_CORE_DIR="$PWD/.platformio"
            export PYTHONPATH="${pioEsptoolPath}''${PYTHONPATH:+:$PYTHONPATH}"

            # 初回だけリポジトリ管理のpre-commit hookを入れる。フック内では複数の
            # nix developが並列に走るため、毎回installすると同じhookのrenameが競合する。
            if git rev-parse --git-dir >/dev/null 2>&1; then
              hook_path="$(git rev-parse --git-path hooks/pre-commit)"
              if [ ! -f "$hook_path" ] || ! grep -q lefthook "$hook_path"; then
                lefthook install >/dev/null
              fi
            fi
          '';
        };

        formatter = pkgs.nixpkgs-fmt;

        # flake check 自体でも、開発に必須なツール集合が評価・実行できることを確認する。
        checks.toolchain = pkgs.runCommand "compass-stackchan-toolchain-check"
          {
            nativeBuildInputs = with pkgs; [
              actionlint
              clang-tools
              cmake
              gitleaks
              just
              pinact
              python3
            ];
          } ''
          {
            just --version
            cmake --version
            clang-tidy --version
            python3 --version
            actionlint --version
            gitleaks version
            pinact version
          } > "$out"
        '';
      });
}
