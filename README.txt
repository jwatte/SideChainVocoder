
This is release 1.0 of SideChainVocoder.
It is a VST plug-in for audio hosts on 32-bit windows, such as CuBase, FruityLoops, etc 
(although it has only been tested with CuBase Studio 4.1 for now).
This binary, and the sources, are free to distribute, as long as you agree to hold the 
author harmless for any damages arising out of your use or lack of use of the program.

The home page for SideChainVocoder is http://vst3.googlecode.com/

To install, drop the dll in a plug-ins folder for your VST host, or in a new folder and 
configure the host to load plug-ins from that folder.
You will also need to install the visual studio 2005 sp1 runtime libraries from the 
following URL:  
http://www.microsoft.com/downloads/details.aspx?familyid=200b2fd9-ae1a-4a14-984d-389c36f85647&displaylang=en

Again: THIS PLUGIN WON'T WORK WITHOUT THE VC2005 SP1 RUNTIME FOUND AT THE URL ABOVE!

After you add the vocoder to a stereo track, it will vocode the left channel using the 
right channel as a modulator, and vocode the right channel using the left channel as a 
modulator, using a 13-band FFT-based vocoder. There is 128 samples of latency. Currently, 
the vocoder does not adjust to higher sample rates, so although it will work, it will 
likely be too coarse/grainy when running at 96 or 192 kHz sampling rates.

Please file any suggestions or comments at http://vst3.googlecode.com/

