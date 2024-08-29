FROM python:3.9-slim-bullseye

RUN apt update && apt install -y nmap

# Create default user folder
RUN mkdir -p /home/pi

# Install dvl service
COPY dvl-a50 /home/pi/dvl-a50
RUN cd /home/pi/dvl-a50 && pip3 install .

LABEL version="1.0.7"
LABEL permissions='\
{\
 "ExposedPorts": {\
   "9001/tcp": {}\
  },\
  "HostConfig": {\
    "Binds":["/root/.config:/root/.config"],\
    "ExtraHosts": [\
      "host.docker.internal:host-gateway"\
    ],\
    "PortBindings": {\
      "9001/tcp": [\
        {\
          "HostPort": ""\
        }\
      ]\
    }\
  }\
}'
LABEL authors='[\
    {\
        "name": "Willian Galvani",\
        "email": "willian@bluerobotics.com"\
    }\
]'
LABEL company='{\
        "about": "",\
        "name": "Blue Robotics",\
        "email": "support@bluerobotics.com"\
    }'
LABEL type="device-integration"
LABEL tags='[\
        "positioning",\
        "navigation",\
        "doppler-velocity-log"\
    ]'
LABEL readme='https://raw.githubusercontent.com/bluerobotics/BlueOS-Water-Linked-DVL/{tag}/README.md'
LABEL links='{\
        "website": "https://github.com/bluerobotics/BlueOS-Water-Linked-DVL",\
        "support": "https://github.com/bluerobotics/BlueOS-Water-Linked-DVL/issues"\
    }'
LABEL requirements="core >= 1.1"

ENTRYPOINT /home/pi/dvl-a50/main.py
