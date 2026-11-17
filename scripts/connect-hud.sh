#!/bin/bash
# Open the presenter HUD through an SSM tunnel -- no inbound ports on the instance.
# The HUD (hud/server.py) listens on the instance loopback, 127.0.0.1:8080.
# Needs: AWS CLI + session-manager-plugin, credentials for the sim account.
set -euo pipefail
I=${SPOT_SIM_INSTANCE:?set SPOT_SIM_INSTANCE to the sim host instance id}; R=${AWS_REGION:-us-west-2}
P=${LOCAL_PORT:-8080}; RP=${REMOTE_PORT:-8080}
# The HUD answers only localhost:<its port> (threat model T2: that is what stops a
# DNS-rebinding page), so the browser must use the same port the HUD listens on.
[ "$P" = "$RP" ] || { echo "LOCAL_PORT ($P) must equal REMOTE_PORT ($RP): the HUD answers only localhost:$RP" >&2; exit 2; }
echo "Browse http://localhost:$P (plain HTTP; the tunnel itself is encrypted). Ctrl-C to close."
echo "Tip: full screen (F11 / Ctrl-Cmd-F) at 1920x1080 for the projector layout."
# SPOT_PORT_FORWARD_DOCUMENT: SpotDemo-PortForward (docs/iam/port-forwarding-document.json,
# 8443 and 8080 only) when you use the shipped presenter policy.
D=${SPOT_PORT_FORWARD_DOCUMENT:-AWS-StartPortForwardingSession}
[[ "$D" =~ ^[A-Za-z0-9_.-]{3,128}$ ]] || { echo "bad SPOT_PORT_FORWARD_DOCUMENT '$D'" >&2; exit 2; }
exec aws ssm start-session --region "$R" --target "$I" --document-name "$D" \
  --parameters "portNumber=$RP,localPortNumber=$P"
