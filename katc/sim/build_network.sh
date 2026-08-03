#!/bin/bash
# Rebuild with explicit geometry options to prevent crossing paths.
set -e
netconvert \
  --node-files intersection.nod.xml \
  --edge-files intersection.edg.xml \
  --connection-files intersection.con.xml \
  --output-file intersection.net.xml \
  --no-turnarounds \
  --tls.guess false \
  --check-lane-foes.all \
  --no-internal-links false \
  --junctions.corner-detail 5
echo "Built intersection.net.xml"
