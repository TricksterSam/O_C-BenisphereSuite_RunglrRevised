Welcome to Phazerville with Relabi App
===

### What's different?

This version modifies Runglbook to be more useful in building a Benjolin. CV2 is automatically XOR against the next bit in the shift register. Out2 provides this XOR value.

This version also includes a new Applet called Relabi, which generates a complex waveform from four LFOs and a gate derived from a threshold on that waveform. Use this app to generate deterministic CV that feels chaotic on Output 1 and a gate that feels like a rhythm that is always slipping the pulse on Output 2. Run the relabi wave through Hemisphere's Schmidt trigger for more broken rhythms. The principle and purpose of relabi (music of the self-erasing pulse) is detailed [here](http://www.johnberndt.org/relabi/). The code is a complete C++ reimagining of Pure Data software designed for creating relabi. One function not yet available include setting the phase of the LFOs which they reset to when receiving a pulse in the first Gate input. Another function not available is the ability to save the current settings of the Relabi app.

