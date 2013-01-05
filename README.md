SprayWind
=========

Remote wind and weather sensor that can use a satellite link to send information back to a website.


## Overview

This project uses an anemometer to meassure the wind. An arduino counts the pulses from the anemometer to compute a wind speed. It then uses a Spot Connect device to transmit this wind and temperature information to the web via the globalstar satalite system. This generates an email that updates the data on a django based website. All the code is open source. 

This project was started as an little amusement to do over the holidays and provide information about the weather conditioins at Spray Lakes in Alberta Canada for the the folks that snowkite there. You can see the current data from this station at spraywind.com. I look forward to hearing about what other people end up doing with this system.



## Design 

Some documentation of the design can be found at
(http://doc.spraywind.com/doc/design.html)


## Tasks and Bug tracker

Have a bug or a feature request? [Please open a new issue](https://github.com/fluffy/SprayWind/issues).


## Contributors

**Cullen (Fluffy) Jennings**


## Copyright and license

Copyright (c) 2012, Cullen Jennings <fluffy@iii.ca> All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 
 
 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer. 
 
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution. 
 
 3. Neither the name of Cullen Jennings nor the names of its contributors may 
 be used to endorse or promote products derived from this software without 
 specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,::w BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
