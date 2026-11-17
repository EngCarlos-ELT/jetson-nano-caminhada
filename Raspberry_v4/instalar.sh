#!/bin/bash
# Instala o mestre do exo na Raspberry Pi 4 sem criar regra sudoers.
# Rode dentro da pasta Raspberry_v4: sudo bash instalar.sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "Rode com sudo: sudo bash instalar.sh" >&2
    exit 1
fi

PASTA=$(cd "$(dirname "$0")" && pwd)
DONO=$(stat -c '%U' "$PASTA")
if [ "$DONO" = "root" ] || [ -z "$DONO" ]; then
    echo "A pasta nao pode pertencer ao root. Copie para o usuario da Pi." >&2
    exit 1
fi

apt-get update
apt-get install -y python3-lgpio python3-serial
usermod -a -G dialout,gpio "$DONO"

cat > /etc/systemd/system/exo-master.service <<EOF
[Unit]
Description=Mestre do exo na Raspberry Pi 4: LED de pronto e caminhada por botao
After=local-fs.target

[Service]
Type=simple
User=$DONO
SupplementaryGroups=dialout gpio
WorkingDirectory=$PASTA
ExecStart=/usr/bin/python3 -u $PASTA/exo_master.py
Restart=always
RestartSec=2
KillSignal=SIGTERM
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable exo-master.service
systemctl restart exo-master.service
echo "Mestre instalado para $DONO em $PASTA. A caminhada entra direto; o LED acende em pe."
echo "Este instalador nao configura sudoers. Revise localmente a permissao de desligamento por botao."
