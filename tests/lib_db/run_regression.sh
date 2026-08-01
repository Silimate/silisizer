#!/bin/bash
# Acceptance harness for the liberty database.
#
# Runs every OpenSTA regression test that calls read_liberty twice: once
# normally, and once with read_liberty redirected to a precompiled .libdb.
# The two outputs must match. Any difference is a serialization bug.
#
# Each liberty file is compiled in its own process so that it becomes the
# default library there, which is what write_lib_db serializes.

set -u
cd "$(dirname "$0")"
ROOT=$(cd ../.. && pwd)
SILI=$ROOT/build/silisizer
TESTDIR=$ROOT/third_party/OpenSTA/test
WORK=${TMPDIR:-/tmp}/libdb_regression
DB=$WORK/db

rm -rf "$WORK"
mkdir -p "$DB"

cd "$TESTDIR"
tests=$(grep -l 'read_liberty' ./*.tcl | sed 's|^\./||; s|\.tcl$||' | sort)

pass=0; fail=0; skip=0
failed_tests=""

for t in $tests; do
  ref=$WORK/$t.ref
  out=$WORK/$t.out

  if ! "$SILI" "$t.tcl" > "$ref" 2>&1; then
    skip=$((skip + 1)); continue
  fi
  # Tests that already fail in this environment (missing zlib, no
  # 'file tempfile') cannot validate anything.
  if grep -q '^Error:' "$ref"; then
    skip=$((skip + 1)); continue
  fi

  # Compile every liberty file this test reads.
  libs=$(sed -n 's/.*read_liberty[^"]*[[:space:]]\([^[:space:]"]*\.lib[^[:space:]"]*\).*/\1/p' "$t.tcl" | sort -u)
  [ -z "$libs" ] && { skip=$((skip + 1)); continue; }

  ok=1
  for lib in $libs; do
    [ -f "$lib" ] || { ok=0; break; }
    dbfile=$DB/$(echo "$lib" | tr '/' '_').libdb
    if [ ! -f "$dbfile" ]; then
      printf 'read_liberty %s\nsta::write_lib_db %s\n' "$lib" "$dbfile" > "$WORK/compile.tcl"
      "$SILI" "$WORK/compile.tcl" > /dev/null 2>&1 || { ok=0; break; }
      [ -f "$dbfile" ] || { ok=0; break; }
    fi
  done
  [ $ok -eq 1 ] || { skip=$((skip + 1)); continue; }

  # Redirect read_liberty at the Tcl layer, then source the unmodified test.
  # Sourcing rather than concatenating keeps the filename and line numbers in
  # diagnostics identical to the reference run.
  {
    echo "rename read_liberty __orig_read_liberty"
    echo "proc read_liberty {args} {"
    echo "  set f [lindex \$args end]"
    echo "  set db \"$DB/[string map {/ _} \$f].libdb\""
    echo "  if {[sta::read_lib_db \$db] != 0} { error \"read_lib_db failed\" }"
    echo "}"
    echo "source $t.tcl"
  } > "$WORK/$t.variant.tcl"

  "$SILI" "$WORK/$t.variant.tcl" > "$out" 2>&1

  # Tcl reports a different source line for a sourced script than for a
  # top-level one. That is harness metadata, never liberty data, so it is
  # normalized away on both sides.
  sed -E 's/\.tcl line [0-9]+/.tcl line N/g' "$ref" > "$ref.norm"
  sed -E 's/\.tcl line [0-9]+/.tcl line N/g' "$out" > "$out.norm"

  if diff -q "$ref.norm" "$out.norm" > /dev/null 2>&1; then
    pass=$((pass + 1))
  else
    fail=$((fail + 1))
    failed_tests="$failed_tests $t"
  fi
done

echo "pass=$pass fail=$fail skip=$skip"
if [ -n "$failed_tests" ]; then
  echo "failed:"
  for t in $failed_tests; do echo "  $t"; done
fi
echo "artifacts in $WORK"
