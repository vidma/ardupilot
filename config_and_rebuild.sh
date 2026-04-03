# git clone --recurse-submodules git@github.com:vidma/ardupilot.git
#./waf configure --board=MambaF405-2022 && ./waf clean && ./waf plane

# ./waf configure --board=Pixhawk6X && ./waf clean && ./waf plane
./waf configure --board=MicoAir743-Lite && ./waf clean && ./waf plane
