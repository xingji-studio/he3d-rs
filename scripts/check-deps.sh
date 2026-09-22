#!/usr/bin/env bash
#
# Verify that every crate cargo unpacked into the registry still matches the
# checksummed `.crate` archive it was unpacked from.
# 校验 cargo 解压到 registry 里的每个 crate 是否仍与它来源的、带校验和的
# `.crate` 压缩包一致。
#
# Why this exists / 为什么需要它
# -----------------------------
# cargo verifies the checksum of the archive it downloads, but it never verifies
# the files it unpacks from it. Nothing stops an editor, an IDE, a code-cleanup
# pass, or a script from rewriting a dependency's source in place — the build
# then fails with an error that points at the dependency instead of at the real
# cause, which is a corrupted local cache.
# cargo 会校验下载的压缩包的校验和，但**从不校验**从它解压出来的文件。编辑器、
# IDE、代码清理工具或脚本可以就地改写依赖源码；随后构建失败，报错指向依赖本身，
# 而真正的原因是本地缓存被改坏了。
#
# Usage / 用法
# ------------
#   scripts/check-deps.sh            verify every unpacked crate
#   scripts/check-deps.sh --quiet    print only mismatches and the summary
#
# Exit status / 退出码
# --------------------
#   0  every unpacked crate matches its archive
#   1  at least one crate differs, or an archive is missing
#
# Repair / 修复
# -------------
# To restore every unpacked crate from its (verified) archive, delete the
# unpacked tree and let cargo rebuild it — no network access is needed:
# 要从（已校验的）压缩包还原全部解压内容，删掉解压目录让 cargo 重新解压即可，
# 全程无需联网：
#
#   rm -rf "${CARGO_HOME:-$HOME/.cargo}/registry/src"
#   cargo fetch
#
set -uo pipefail

CARGO_HOME="${CARGO_HOME:-$HOME/.cargo}"
SRC_ROOT="$CARGO_HOME/registry/src"
CACHE_ROOT="$CARGO_HOME/registry/cache"

quiet=0
if [ "${1:-}" = "--quiet" ]; then
	quiet=1
fi

if [ ! -d "$SRC_ROOT" ]; then
	echo "no unpacked registry sources at $SRC_ROOT — nothing to check" >&2
	exit 0
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

checked=0
tampered=0
missing=0

for crate_dir in "$SRC_ROOT"/*/*/; do
	[ -d "$crate_dir" ] || continue
	name="$(basename "${crate_dir%/}")"

	archive=""
	for candidate in "$CACHE_ROOT"/*/"$name.crate"; do
		if [ -f "$candidate" ]; then
			archive="$candidate"
			break
		fi
	done
	if [ -z "$archive" ]; then
		# A git or path dependency has no `.crate` archive to compare against.
		# git 或 path 依赖没有 `.crate` 压缩包可供比对。
		printf 'no-archive  %s\n' "$name"
		missing=$((missing + 1))
		continue
	fi

	rm -rf "$tmp/$name"
	if ! tar -xzf "$archive" -C "$tmp" 2>/dev/null; then
		printf 'unreadable  %s\n' "$name"
		tampered=$((tampered + 1))
		continue
	fi

	# cargo rewrites `Cargo.toml` as it unpacks and drops a `.cargo-ok` marker,
	# so neither can ever match the archive. Every other file must match byte
	# for byte.
	# cargo 解压时会重写 `Cargo.toml` 并写入 `.cargo-ok` 标记，二者都不可能
	# 与压缩包一致；其余文件必须逐字节一致。
	report="$(diff -rq --exclude=.cargo-ok --exclude=Cargo.toml "$tmp/$name" "$crate_dir" 2>&1)"
	if [ -n "$report" ]; then
		echo "=== TAMPERED: $name ==="
		echo "$report"
		tampered=$((tampered + 1))
	else
		if [ "$quiet" -eq 0 ]; then
			printf 'ok          %s\n' "$name"
		fi
	fi
	checked=$((checked + 1))
done

echo
echo "checked=$checked tampered=$tampered no-archive=$missing"

if [ "$tampered" -eq 0 ] && [ "$missing" -eq 0 ]; then
	exit 0
fi
exit 1
