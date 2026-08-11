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
