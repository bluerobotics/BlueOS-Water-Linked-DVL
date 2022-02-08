# BlueOS-dvl

This is a docker implementation of a Waterlinked DVL-a50 driver for the new Blue Robotics BlueOS.

To set this up, ssh into the Raspberry Pi (or access via `red-pill` in [BlueOS Terminal](https://docs.bluerobotics.com/ardusub-zola/software/onboard/BlueOS-1.0/advanced-usage/#terminal)) and run

`sudo docker run -d --net=host -v /root/.config/blueos:/root/.config --name=BlueOS-dvl-all-in-one --restart=unless-stopped williangalvani/BlueOS-dvl:all-in-one
`

The service will show in the "Available Services" section in BlueOS, where there are some configuration options.
