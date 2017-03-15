# Introduction

This library is a port from the [**Google Chrome EC library**](https://www.chromium.org/chromium-os/ec-development). The goal is port the code to C++ and packge it as a library that can be used with Arduino, Particle, and other embedded applications. This code is very much a work in progress. Not all of it has been tested. 

USB Power Delivery is what is used to do things with USB-C including negotiating for higher voltages, entering alternate modes, and swapping power roles. This library requires a lower-level library implementing the USB Type-C Port Manager (TCPM) interface. For now, it is hard-coded to use the [**FUSB302 library**](https://github.com/ReclaimerLabs/FUSB302/) only. 

# Example Usage

The included example shows some basic usage of this library. The example will negotiate for the highest power available from the source it is plugged into. It will take up to 20V and 100W. The example also implements all the board-specific functions required to make the code work. 

# Next Development Steps

The next steps are to remove the Arduino-specific references to make the code more platform agnostic. At the same time, the intention is pull together all the application-specific functions and settings a developer using this library would need to complete. 

# Questions, Comments, and Contributions

Pull requests are welcome. If you have questions or comments, you can email me directly at jason@reclaimerlabs.com. 
