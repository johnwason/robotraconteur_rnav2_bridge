#!/usr/bin/env bash
# Add (or replace) an RR user in certs/users.txt (git-ignored) for RR_NAV2_AUTH=1.
# Usage: scripts/add_user.sh <username>     (password is read from the terminal)
# Format (RR PasswordFileUserAuthenticator): "<user> <md5(password)> <privileges>".
# MD5 is what RR's password file supports; protect the file and use TLS on networks.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SECRETS="${RR_NAV2_SECRETS_DIR:-$ROOT/certs}"
user="${1:?usage: add_user.sh <username>}"
[[ "$user" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "invalid username" >&2; exit 1; }
read -r -s -p "Password for $user: " pw; echo
read -r -s -p "Repeat: " pw2; echo
[[ "$pw" == "$pw2" && -n "$pw" ]] || { echo "passwords differ or empty" >&2; exit 1; }
hash="$(printf '%s' "$pw" | md5sum | cut -d' ' -f1)"
mkdir -p "$SECRETS"; chmod 700 "$SECRETS"
touch "$SECRETS/users.txt"; chmod 600 "$SECRETS/users.txt"
grep -v "^$user " "$SECRETS/users.txt" > "$SECRETS/users.txt.new" || true
echo "$user $hash none" >> "$SECRETS/users.txt.new"
mv "$SECRETS/users.txt.new" "$SECRETS/users.txt"
echo "user '$user' written to $SECRETS/users.txt"
