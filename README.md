# ads1220-python
A library for Texas Instruments' ADS1220 running with RPi, written in Python

---

The library is focused on usage with a Raspberry Pi as it makes use of `spidev` and `RPi.GPIO` packages. Besides this it is just based on the official datasheet of ADS1220 provided by Texas Instruments (find here)[https://www.ti.com/lit/ds/symlink/ads1220.pdf] or the datasheet version used for this library.

## Usage   
No license up to now: The library can be freely used and modified, but the credits/author information about me should be kept or at least some credits placed somewhere else.

---

In your own python skripts, just make an import for this library:
```python
from ads1220 import ADS1220
```

Then declare a variable and initialize it with a new instance of ADS1220 class. Mandatory initializer methods for GPIO, SPI and ADC config are called in class constructor.
```python
# Init ADS1220
adc = ADS1220()
```
Configuration is completely handled in `config_ADC` method. The ADS1220 uses 4 16-bit configuration registers, configuration options are then encoded in a bitmask. See the method, all configuration constants `CFGx` at the beginning of the ADS1220 class or refer to the datasheet for more and detailed information. ADC configuration can be omitted, the device will then operate with default values (all bitmasks are 0x0000).

## Contribution
If you find any bugs or issues, please open an issue for further discussion. For improvement proposals you can also open an issue or make a pull request. Any constructive criticism is welcome! 
