#!/usr/bin/env bash
# Build the fat jar, push it to EC2, install/restart the systemd service.
#
#   EC2_HOST=<public-ip> EC2_KEY=~/.ssh/<key>.pem ./deploy/deploy.sh
#
# Override EC2_USER for a non Amazon Linux AMI (ubuntu, admin, ...).
set -euo pipefail

: "${EC2_HOST:?set EC2_HOST to the instance public IP or DNS}"
: "${EC2_KEY:?set EC2_KEY to the .pem private key path}"
EC2_USER="${EC2_USER:-ec2-user}"
REMOTE_DIR="${REMOTE_DIR:-/opt/stacksense}"
APP_PORT="${APP_PORT:-8080}"
RENDERED="$(mktemp -t stacksense.service)"
trap 'rm -f "$RENDERED"' EXIT

cd "$(dirname "$0")/.."

echo "==> building"
mvn -q clean package

SSH=(ssh -i "$EC2_KEY" -o StrictHostKeyChecking=accept-new "$EC2_USER@$EC2_HOST")

echo "==> checking java on the instance"
"${SSH[@]}" 'java -version' || {
  echo "java missing. install it first, e.g.:"
  echo "  sudo dnf install -y java-11-amazon-corretto-headless   # Amazon Linux 2023"
  echo "  sudo apt install -y openjdk-11-jre-headless            # Ubuntu"
  exit 1
}

echo "==> uploading"
"${SSH[@]}" "sudo mkdir -p $REMOTE_DIR && sudo chown $EC2_USER $REMOTE_DIR"
scp -i "$EC2_KEY" target/app.jar "$EC2_USER@$EC2_HOST:$REMOTE_DIR/app.jar"

REMOTE_JAVA=$("${SSH[@]}" 'command -v java')
sed -e "s|__USER__|$EC2_USER|g" \
    -e "s|__DIR__|$REMOTE_DIR|g" \
    -e "s|__JAVA__|$REMOTE_JAVA|g" \
    -e "s|__PORT__|$APP_PORT|g" \
    deploy/stacksense.service > "$RENDERED"
scp -i "$EC2_KEY" "$RENDERED" "$EC2_USER@$EC2_HOST:/tmp/stacksense.service"

echo "==> installing service"
"${SSH[@]}" "sudo mv /tmp/stacksense.service /etc/systemd/system/stacksense.service \
  && sudo systemctl daemon-reload \
  && sudo systemctl enable stacksense \
  && sudo systemctl restart stacksense"

sleep 3
echo "==> status"
"${SSH[@]}" "systemctl is-active stacksense; curl -s --max-time 5 localhost:$APP_PORT/api/health; echo"

echo
echo "local reachability check:"
curl -s --max-time 8 "http://$EC2_HOST:$APP_PORT/api/health" && echo " <- reachable" \
  || echo " <- NOT reachable from here: open TCP $APP_PORT in the security group"
