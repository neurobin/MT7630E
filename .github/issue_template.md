# System info

Entry | Details
----- | -------
OS | <put your os name here>
Kernel version | <kernel version here (the output of `uname -r`)>
New install | <yes/no>
DKMS | <yes/no>
Compiler | < include version e.g: gcc-5.4.0 >

# Devince info

**Device ID:** <run `lsusb |grep -i 'mediatek\|foxcon'` to get it (e.g *0e8d:763f*)>

**General info:**

```
#block starts
run 'lspci -vv | grep -A20 -i mediatek' and paste the output in this block
#block ends
```

**Bluetooth stats:**

```
#block starts
run 'rfkill list' and paste the output in this block
#block ends
```

# What didn't work?
[please be specific. Do not post excessive error message (only the part that makes sense), you can post the complete error message in the 'Error code' section below.]







# What did you try?
[please include all details, every small step is important, if already explained above, just use ^^^ or 'see above']


# Error code:

```
#block starts
[please include error code in this block]
#block ends
```

# Additional info
[Put any additional information here or leave it blank]
