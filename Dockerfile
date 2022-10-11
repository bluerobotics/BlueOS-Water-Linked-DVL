FROM python:3.9-slim-bullseye

RUN apt update && apt install -y nmap

# Create default user folder
RUN mkdir -p /home/pi

# Install dvl service
COPY dvl-a50 /home/pi/dvl-a50
RUN cd /home/pi/dvl-a50 && pip3 install .

LABEL version="1.0"
LABEL permissions '\
{\
    "NetworkMode": "host"\
}'
LABEL authors '[\
    {\
        "name": "Willian Galvani",\
        "email": "willian@bluerobotics.com"\
    }\
]'
LABEL docs ''
LABEL company '{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL readme 'https://raw.githubusercontent.com/bluerobotics/BlueOS-Water-Linked-DVL/{tag}/README.md'
LABEL website 'https://github.com/bluerobotics/BlueOS-Water-Linked-DVL'
LABEL support 'https://github.com/bluerobotics/BlueOS-Water-Linked-DVL'
LABEL requirements "core >  1"

ENTRYPOINT /home/pi/dvl-a50/main.py
