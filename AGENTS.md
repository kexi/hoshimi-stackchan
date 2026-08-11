# エージェント向け指示

## コマンド実行

- `justfile` に同等のレシピが定義されている操作は、下位コマンドを直接実行せず
  `just` 経由で実行する。たとえば `pio run` を直接実行せず、`just build`、
  `just upload`、`just set-time` を使う。
