default:
    @just --list

# --- ホストビルド (ESP32 非依存コア + テスト) ---

configure:
    cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

compile: configure
    cmake --build build/host

test-host: compile
    ctest --test-dir build/host --output-on-failure

# --- 整形・静的解析 ---

fmt:
    rg --files lib test -g '*.cpp' -g '*.hpp' | xargs clang-format -i
    rg --files firmware/src -g '*.cpp' -g '*.h' | xargs clang-format -i

fmt-check:
    rg --files lib test -g '*.cpp' -g '*.hpp' | xargs clang-format --dry-run --Werror
    rg --files firmware/src -g '*.cpp' -g '*.h' | xargs clang-format --dry-run --Werror

lint: configure
    run-clang-tidy -p build/host -j 2 -quiet

# --- ファームウェア ---

build:
    pio run --project-dir firmware

# ホストの時計で実機の RTC を合わせて書き込む。
# Wi-Fi が無い場所でも天体を指せるようにするため、ビルド時刻を埋め込む。
set-time:
    #!/usr/bin/env bash
    set -euo pipefail
    port=$(pio device list --json-output \
      | python3 -c 'import json,sys; print(next((d["port"] for d in json.load(sys.stdin) if "303A:1001" in (d.get("hwid") or "")), ""))')
    if [ -z "$port" ]; then
      echo "CoreS3 が見つかりません (hwid 303A:1001)" >&2
      exit 1
    fi
    # 書き込みに約 30 秒かかるぶんを見込んで先の時刻を入れる
    now=$(python3 -c 'import time; print(int(time.time()) + 35)')
    PLATFORMIO_BUILD_FLAGS="-DBUILD_UNIX_TIME=${now}" \
      pio run --project-dir firmware --target upload --upload-port "$port"
    echo "RTC を $(date -r "$now" '+%Y-%m-%d %H:%M:%S') に合わせました" 

upload port='':
    pio run --project-dir firmware --target upload {{ if port == '' { '' } else { '--upload-port ' + port } }}

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

# 実機の計測結果を NVS から回収する。
# この個体は USB CDC シリアルが列挙されないため、ファームは結果を NVS に書き、
# ここでフラッシュごと吸い出して読む (シリアルに頼らずログが取れる)
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
    python3 scripts/watch_serial.py "$port" {{ seconds }}

# 実機の動作を検証する。キャリブレーション・時刻・方位・ターゲット切り替えを
# 順に確かめ、足りないものを具体的に出す。
verify:
    #!/usr/bin/env bash
    set -euo pipefail
    port=$(pio device list --json-output \
      | python3 -c 'import json,sys; print(next((d["port"] for d in json.load(sys.stdin) if "303A:1001" in (d.get("hwid") or "")), ""))')
    if [ -z "$port" ]; then
      echo "CoreS3 が見つかりません (hwid 303A:1001)" >&2
      exit 1
    fi
    python3 scripts/verify_device.py "$port"

clean:
    pio run --project-dir firmware --target clean
    rm -rf build

# --- サプライチェーン・秘密情報 ---

actionlint:
    actionlint

# --check: 未ピンの action があれば fail / --verify: SHA が注記のタグと一致するかリモート照合
pinact:
    pinact run --check --verify --min-age 1

pin-actions:
    pinact run --min-age 1

actions: actionlint pinact

secrets:
    gitleaks git --redact

secrets-staged:
    gitleaks git --pre-commit --staged --redact

# CI の入口。ローカルと CI で同じ recipe を通す。
all: fmt-check lint actions secrets test-host build
