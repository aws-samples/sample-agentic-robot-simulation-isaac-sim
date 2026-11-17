#!/bin/bash
# Open the Isaac Sim desktop (NICE DCV) through an SSM tunnel -- no inbound ports on the instance.
# Needs: AWS CLI + session-manager-plugin, credentials for the sim account.
set -euo pipefail
I=${SPOT_SIM_INSTANCE:?set SPOT_SIM_INSTANCE to the sim host instance id}; R=${AWS_REGION:-us-west-2}; P=${LOCAL_PORT:-8443}
S=${SPOT_DCV_SECRET_ID:-spot-demo/dcv-password}  # the secret you created for the DCV login (README, "AWS resources")
echo "DCV login user: ubuntu. Password: aws secretsmanager get-secret-value --region $R --secret-id $S --query SecretString --output text"
echo "Browse https://localhost:$P (self-signed cert) or use the DCV client with localhost:$P. Ctrl-C to close."
# SPOT_PORT_FORWARD_DOCUMENT: SpotDemo-PortForward (docs/iam/port-forwarding-document.json,
# 8443 and 8080 only) when you use the shipped presenter policy.
D=${SPOT_PORT_FORWARD_DOCUMENT:-AWS-StartPortForwardingSession}
[[ "$D" =~ ^[A-Za-z0-9_.-]{3,128}$ ]] || { echo "bad SPOT_PORT_FORWARD_DOCUMENT '$D'" >&2; exit 2; }
exec aws ssm start-session --region "$R" --target "$I" --document-name "$D" \
  --parameters "portNumber=8443,localPortNumber=$P"
