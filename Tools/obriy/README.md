# Obriy tools

Build SITL for the obriy branch:

```sh
./waf configure --board sitl
./waf copter
```

Run the simulator:

```sh
Tools/autotest/sim_vehicle.py -v ArduCopter --map --console
```
