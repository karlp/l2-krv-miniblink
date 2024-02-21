## USB host on nxp k64/k70

```
$ git submodule update --init
$ scons
$ use one of the elf files for the board you're testing against.
```

This was a hardware validation project only, there is sitll plenty of dangling bits from earlier experiments.

Key takeaways:
* V2400 usb hardware works just fine.
* Don't start tinyusb straight away, _or_ hold the hub in reset first.  Bringing them both up together seems to be unreliable.
* tinyusb host khci still has bugs, especially related to LS devices like keyboards and mice.

