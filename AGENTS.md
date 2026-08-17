# Repository Guidelines

## Project Structure & Module Organization

`firmware/src/` はCoreS3固有の入出力、Wi-Fi、サーボ制御を担当します。ESP32に依存しない
C++17ロジックは `lib/` に分離され、`astro_core`、`compass_core`、`pointing_core`、
`connectivity_core`、`app_core` で構成されます。ホスト用シミュレータは `host/`、C++テストは
`test/*_test.cpp`、Pythonテストは `test/python/test_*.py` に置きます。診断・生成ツールは
`scripts/`、PlatformIO設定は `firmware/platformio.ini` にあります。

## Build, Test, and Development Commands

最初に `nix develop` へ入り、`just setup` でフックを導入してください。日常操作は下位コマンドを
直接呼ばず、同等の `just` レシピを使います。

- `just test`: CMake/CTestのホストテストとPython unittestを実行。
- `just check`: 整形、clang-tidy、テスト、ファームウェアビルドを一括確認。
- `just build`: CoreS3ファームウェアをビルド。
- `just upload`: 接続中の実機へ書き込み。既存ファームウェアを置き換えます。
- `just sensors` / `just target moon` / `just verify`: Wi-Fi診断、対象選択、実機検証。

## Coding Style & Naming Conventions

C++はLLVMベースの `.clang-format`（100桁、ポインタは型側）と `.clang-tidy` に従います。
型は `PascalCase`、関数・変数は `camelCase`、定数は `kPascalCase` を使います。条件式には
`isReady` など意味のある名前を付け、関数は早期リターンを優先してください。コードコメントは
実装方法、テストコメントは保証内容、判断コメントは採用しなかった理由を記します。単体Python
ツールはPEP 723メタデータを持たせ、`uv run scripts/<name>.py` で実行可能にします。

## Testing Guidelines

変更したコアには対応する `*_test.cpp` を追加し、実機なしで再現できる計算と状態遷移を固定します。
Pythonは `test_<module>.py` とし、ネットワークやシリアルをモック化します。PR前に最低でも
`just test`、C++変更では `just fmt-check` と `just lint`、実機変更では監督下で `just verify` を
実行してください。

## Commit & Pull Request Guidelines

履歴に合わせてSemantic Commit Messages（例: `fix: correct compass heading`）を使います。
コミット前にメッセージ案を提示してください。PRには目的、主要変更、実行した検証、実機確認の有無、
関連Issueを記載し、表示変更には画像を添えます。

## Security & Agent Workflow

コマンド実行前に `echo $SHELL` を確認し、そのシェルで有効な構文を使います。Wi-Fi認証情報、MAC、
私的な位置情報をコミットやログへ含めないでください。`wifi_config.h.example` のみ共有します。
