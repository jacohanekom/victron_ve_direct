#!/bin/bash
# Integration test: fake MPPT on a PTY + daemon + ctrl-port commands
set -u
cd "$(dirname "$0")/.."

cleanup() { kill $FAKE_PID $DAEMON_PID 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

python3 test/fake_mppt.py >/tmp/pty.txt 2>/tmp/fake.err &
FAKE_PID=$!
sleep 1
PTY=$(head -1 /tmp/pty.txt)
echo "== fake MPPT on $PTY"

sed "s|/dev/ttyUSB0|$PTY|" config.ini | sed 's/^enabled     = true/enabled = false/' > /tmp/test.ini

./build/victron_ve_direct --config /tmp/test.ini >/tmp/out.jsonl 2>/tmp/err.log &
DAEMON_PID=$!
sleep 2

echo "== first JSON frame:"
head -1 /tmp/out.jsonl | cut -c1-200; echo

ctrl() {  # send one command to ctrl port, print reply
    echo "----- $* -----"
    printf '%s\n' "$*" | timeout 30 python3 -c '
import socket, sys
cmd = sys.stdin.read()
s = socket.create_connection(("127.0.0.1", 8562), timeout=30)
s.sendall(cmd.encode())
data = b""
while True:
    b = s.recv(4096)
    if not b: break
    data += b
print(data.decode(), end="")
'
}

ctrl ping
ctrl "get absorption_voltage"
ctrl "get 0xEDAB"
ctrl "set absorption_voltage 14.60"
ctrl "get absorption_voltage"
ctrl "set absorption_voltage 20.00"      # fake device rejects >16V -> parameter error
ctrl "set load_mode 1"
ctrl "get load_output_state"             # unsupported on fake -> device flag
ctrl "set device_state 3"                # read-only -> local refusal
ctrl "get nonsense"
ctrl "setraw 0xED9C u16 1150"
ctrl "get load_switch_low_level"
echo "----- settings -----"
ctrl settings
echo "== JSON still flowing after hex traffic:"
sleep 1.5
tail -1 /tmp/out.jsonl | cut -c1-160; echo
echo "== parser errors (if any):"
grep -a "Parser" /tmp/err.log | head -5 || true
echo "== DONE"
