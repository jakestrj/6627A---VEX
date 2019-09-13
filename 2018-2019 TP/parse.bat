@ECHO off
python -m readme2tex --nocdn --username jakestrj --project 6627A---VEX --output README.md _README.md && python svg2png.py "svgs/"