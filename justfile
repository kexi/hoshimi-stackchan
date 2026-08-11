default:
    @just --list

# --- 開発環境 ---

setup:
    lefthook install

doctor:
    @git --version
    @just --version
    @cmake --version | head -1
    @clang-tidy --version | head -1
    @pio --version
    @python3 --version
    @uv --version
    @gitleaks version
    @pinact version
    @actionlint --version

update:
    nix flake update

# --- ホストビルド (ESP32 非依存コア + テスト) ---

configure:
    cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

compile: configure
    cmake --build build/host

test-host: compile
    ctest --test-dir build/host --output-on-failure

test-python:
    python3 -m unittest discover -s test/python -p 'test_*.py'

test: test-host test-python

# JPL Horizonsの公開データから、実機へ埋め込む高精度暦DBと独立検証値を更新する。
# 通常のbuild/testは生成済みファイルを使うため、ネット接続は不要。
update-ephemeris:
    uv run scripts/generate_ephemeris_db.py

# --- 整形・静的解析 ---

fmt:
    rg --files lib test -g '*.cpp' -g '*.hpp' | xargs clang-format -i
    rg --files firmware/src -g '*.cpp' -g '*.h' | xargs clang-format -i

fmt-check:
    rg --files lib test -g '*.cpp' -g '*.hpp' | xargs clang-format --dry-run --Werror
    rg --files firmware/src -g '*.cpp' -g '*.h' | xargs clang-format --dry-run --Werror

lint: configure
    run-clang-tidy -p build/host -j 2 -quiet

clang-format: fmt

clang-format-check: fmt-check

clang-tidy: lint

justfile-fmt-check:
    just --fmt --check

justfile-fmt:
    just --fmt

# --- ファームウェア ---

build:
    pio run --project-dir firmware

# ファームウェアを書き込んだ後、ホストの現在時刻をUSB経由でRTCへ同期する。
# 書き込み時間の予測値は使わず、CoreS3の設定完了応答まで確認する。
set-time:
    #!/usr/bin/env bash
    set -euo pipefail
    port=$(pio device list --json-output \
      | python3 -c 'import json,sys; print(next((d["port"] for d in json.load(sys.stdin) if "303A:1001" in (d.get("hwid") or "")), ""))')
    if [ -z "$port" ]; then
      echo "CoreS3 が見つかりません (hwid 303A:1001)" >&2
      exit 1
    fi
    pio run --project-dir firmware --target upload --upload-port "$port"
    uv run scripts/set_device_time.py "$port"

upload port='':
    pio run --project-dir firmware --target upload {{ if port == '' { '' } else { '--upload-port ' + port } }}

flash port='':
    just upload "{{ port }}"

# CoreS3 は /dev/cu.debug-console も見えるので、USB シリアル (303A:1001) だけを掴む
monitor:
    #!/usr/bin/env bash
    set -euo pipefail
    port=$(pio device list --json-output \
      | python3 -c 'import json,sys; print(next((d["port"] for d in json.load(sys.stdin) if "303A:1001" in (d.get("hwid") or "")), ""))')
    if [ -z "$port" ]; then
      echo "CoreS3 が見つかりません (hwid 303A:1001)" >&2
      exit 1
    fi
    pio device monitor --project-dir firmware --port "$port"

# 起動直後の診断値をNVSから回収する。読み出しは実機をリセットするため、
# 動作中の追跡にはwatch/verifyを使い、シリアルが使えない場合の補助に限る。
read-nvs filter='':
    #!/usr/bin/env bash
    set -euo pipefail
    port=$(pio device list --json-output \
      | python3 -c 'import json,sys; print(next((d["port"] for d in json.load(sys.stdin) if "303A:1001" in (d.get("hwid") or "")), ""))')
    if [ -z "$port" ]; then
      echo "CoreS3 が見つかりません (hwid 303A:1001)" >&2
      exit 1
    fi
    PYTHONPATH="$PLATFORMIO_CORE_DIR/packages/tool-esptoolpy:${PYTHONPATH:-}" \
      python3 scripts/read_nvs.py "$port" \
      "$PLATFORMIO_CORE_DIR/packages/tool-esptoolpy/esptool.py" {{ filter }}

# 実機の状態をシリアルから読み続ける。
#
# Why not read-nvs: フラッシュ読み出しは esptool のリセットを伴うので、
# 何度読んでも「起動直後」の値しか取れず、指した結果が観測できない。
watch seconds='30':
    #!/usr/bin/env bash
    set -euo pipefail
    port=$(pio device list --json-output \
      | python3 -c 'import json,sys; print(next((d["port"] for d in json.load(sys.stdin) if "303A:1001" in (d.get("hwid") or "")), ""))')
    if [ -z "$port" ]; then
      echo "CoreS3 が見つかりません (hwid 303A:1001)" >&2
      exit 1
    fi
    uv run scripts/watch_serial.py "$port" {{ seconds }}

logs seconds='30':
    just watch {{ seconds }}

# Wi-FiとmDNSでStack-chanを見つけ、指定した天体へ切り替える。
# 例: just target moon / just target 火星 / just target 7
target target host='':
    uv run scripts/select_target.py {{ quote(target) }} {{ quote(host) }}

# 生のCoreS3軸と顔基準へ変換後の最新センサー値をWi-Fi経由で取得する。
# 例: just sensors / just sensors stackchan-xxxxxx.local
sensors host='':
    uv run scripts/read_sensors.py {{ quote(host) }}

# 傾き許容角を実行中RAMへ適用して校正を再開始する。再flashは不要。
# 例: just calibrate 10 / just calibrate 10 stackchan-xxxxxx.local
calibrate max_tilt='10' host='':
    uv run scripts/configure_calibration.py {{ quote(max_tilt) }} {{ quote(host) }}

# 実機の動作を検証する。キャリブレーション・時刻・方位・ターゲット切り替えを
# 順に確かめ、足りないものを具体的に出す。
verify seconds='120':
    #!/usr/bin/env bash
    set -euo pipefail
    port=$(pio device list --json-output \
      | python3 -c 'import json,sys; print(next((d["port"] for d in json.load(sys.stdin) if "303A:1001" in (d.get("hwid") or "")), ""))')
    if [ -z "$port" ]; then
      echo "CoreS3 が見つかりません (hwid 303A:1001)" >&2
      exit 1
    fi
    uv run scripts/verify_device.py "$port" {{ seconds }}

clean:
    pio run --project-dir firmware --target clean
    rm -rf build

# --- サプライチェーン・秘密情報 ---

actionlint:
    actionlint

# --check: 未ピンの action があれば fail / --verify: SHA が注記のタグと一致するかリモート照合
pinact:
    just pinact-verify

pinact-check:
    pinact run --fix=false --no-api

pinact-verify:
    pinact run --fix=false --verify-comment --verify-min-age --min-age 1

pin-actions:
    pinact run --min-age 1

actions: actionlint pinact-check

secrets-history:
    gitleaks git --redact

secrets-worktree:
    gitleaks dir . --redact

secrets: secrets-history secrets-worktree

secrets-staged:
    gitleaks git --pre-commit --staged --redact

gitleaks: secrets

gitleaks-staged: secrets-staged

# ホスト、ファームウェア、設定の通常品質ゲート。外部APIが必要な検査はciへ分ける。
check: justfile-fmt-check fmt-check lint test build

# CI の入口。ローカルと CI で同じ recipe を通す。
ci: check actionlint pinact-verify secrets

all: ci

# 環境と再現可能な検査結果を、端末固有の無視対象ディレクトリへまとめる。
# seconds > 0 のときだけ、接続中CoreS3の有界ログも追加する。
diagnose seconds='0':
    #!/usr/bin/env bash
    set -euo pipefail
    stamp=$(date -u '+%Y%m%dT%H%M%SZ')
    destination="$PWD/.stackchan/diagnostics/$stamp"
    mkdir -p "$destination"
    set +e
    {
      echo "revision=$(git rev-parse HEAD)"
      git status --short
      just doctor
      just test
      if [ "{{ seconds }}" -gt 0 ]; then
        just watch "{{ seconds }}"
      fi
    } 2>&1 | tee "$destination/diagnose.log"
    status=${PIPESTATUS[0]}
    set -e
    echo "diagnostics=$destination"
    exit "$status"
