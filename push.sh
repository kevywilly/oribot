#!/bin/bash
sudo chown -R orin *
git add -A .
git commit -a -m 'latest'
git push
