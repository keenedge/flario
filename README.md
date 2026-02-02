# flario

## Windows Dev Config

ESP32 plugin to a local com port. (COM10)

A a TASkSchd task
- At System Startup
- Program: "C:\Program Files (x86)\com0com\hub4com.exe"
- Args: --baud=115200 \\.\COM10 --use-driver=tcp 7000

## MX Linux dev config

/etc/systemd/system/ttyVCOM0

   [Unit]
   Description=ESP32 TCP to TTY Bridge (/dev/ttyVCOM0)
   After=network-online.target
   Wants=network-online.target

   [Service]
   Type=simple
   ExecStart=/usr/bin/socat \
     pty,raw,echo=0,link=/dev/ttyVCOM0,mode=666,b115200 \
     tcp:10.0.0.8:7000,retry=1000000,interval=1
   Restart=always
   RestartSec=1

   [Install]
   WantedBy=multi-user.target

### Restart Service
sudo systemctl daemon-reload
sudo systemctl restart ttyvcom0.service

### Test

   screen /dev/ttyVCOM0


