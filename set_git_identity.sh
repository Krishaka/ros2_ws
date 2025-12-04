#!/bin/bash
# Usage:
#  ./set_git_identity.sh email "Full Name" [--global]
# Examples:
#  ./set_git_identity.sh you@example.com "Your Name"
#  ./set_git_identity.sh you@example.com "Your Name" --global

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 email \"Full Name\" [--global]"
  exit 1
fi

EMAIL="$1"
NAME="$2"
GLOBAL_FLAG=""
if [ "$3" = "--global" ]; then
  GLOBAL_FLAG="--global"
fi

git config $GLOBAL_FLAG user.email "$EMAIL"
git config $GLOBAL_FLAG user.name "$NAME"

echo "Git identity set:"
git config $GLOBAL_FLAG user.email && git config $GLOBAL_FLAG user.name
