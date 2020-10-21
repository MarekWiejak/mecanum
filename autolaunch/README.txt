place astrocent.service at /lib/systemd/system directory
place startup_launch.sh at ~/scripts directory

RUN:
cd ~/scripts
touch startup_launch.sh
chmod +x startup_launch.sh
sudo systemctl daemon-reload

to enable on boot:
sudo systemctl enable astrocent.service

to disable:
sudo systemctl disable astrocent.service
