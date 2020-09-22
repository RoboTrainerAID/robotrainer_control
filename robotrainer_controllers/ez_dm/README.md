# EZDM (Easy Drive Mode)

An easier way of calling the drive mode service according to wheel positions.

Example:
```
python3 diff.py fr fl
```
Sets differential mode so, that the virtual axle goes through the front right
(fr) and front left (fl) wheels.

Every script in EZDM prints its usage message.

Not every behaviour is achievable through EZDM.

Units are meters and radians, because this is what the robotrainer_controller 
uses.

## What the files do:

### ros_srv_caller.sh

* The actual "rosservice call ..." command resides here. The Python script calls
  this bash script with appropriate parameters.

### common.py

* Wheel positions are defined here. (WHEELS dictionary).  As this README was
  written, wheel positions for sr2 and sr3 platforms were in it.
   * fl: front left wheel of sr2
   * fr: front right wheel of sr2
   * bl: rear (back) left wheel of sr2
   * br: rear (back) right wheel of sr2
   * f: front wheel of sr3
   * l: left wheel of sr3
   * r: right wheel of sr3
* Common tasks such as checking for appropriate arguments are here.
* `python3 common.py` prints parameters of virtual axles for all possible wheel
  pairs, in case this is ever needed.

### diff.py

Sets differential mode so, that the virtual axle goes through specified wheels.

Example:
```
python3 diff.py fr fl
```

### ack.py

Sets Ackermann mode so, that the virtual axle goes through specified wheels. The
first argument is the minimum turning radius.

Example:
```
python3 ack.py 1.2 fr fl
```

### ack_old.py

A variation of the previous mode. Both this and the previous mode maps the
rotational speed into an interval. Previous mode does so by scaling. This mode
does so by cutting of the excess speed. 

### ask_current_state.py

Echoes the currently set mode and its parameters.

Example:
```
python3 ask_current_state.py
```

### pivot.py

Fix ICR on a wheel.

Example:
```
python3 pivot.py fl
```

### omni.py

Back to the default mode.

Example:
```
python3 omni.py
```
