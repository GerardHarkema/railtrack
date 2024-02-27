python3 generate_services_from_template.py
sudo cp railtrack_control.service /etc/systemd/system/

sudo systemctl enable railtrack_control.service
